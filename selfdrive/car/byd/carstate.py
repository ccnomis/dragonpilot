from cereal import car
from collections import defaultdict
from common.numpy_fast import interp
from opendbc.can.can_define import CANDefine
from opendbc.can.parser import CANParser
from selfdrive.config import Conversions as CV
from selfdrive.car.interfaces import CarStateBase
from selfdrive.car.byd.values import CAR, DBC, STEER_THRESHOLD, SPEED_FACTOR

def calc_cruise_offset(offset, speed):
  # euristic formula so that speed is controlled to ~ 0.3m/s below pid_speed
  # constraints to solve for _K0, _K1, _K2 are:
  # - speed = 0m/s, out = -0.3
  # - speed = 34m/s, offset = 20, out = -0.25
  # - speed = 34m/s, offset = -2.5, out = -1.8
  _K0 = -0.3
  _K1 = -0.01879
  _K2 = 0.01013
  return min(_K0 + _K1 * speed + _K2 * speed * offset, 0.)


def get_can_signals(CP):
  # this function generates lists for signal, messages and initial values
  signals = [
      ("XMISSION_SPEED", "ENGINE_DATA", 0),
      ("WHEEL_SPEED_FL", "WHEEL_SPEEDS", 0),
      ("WHEEL_SPEED_FR", "WHEEL_SPEEDS", 0),
      ("WHEEL_SPEED_RL", "WHEEL_SPEEDS", 0),
      ("WHEEL_SPEED_RR", "WHEEL_SPEEDS", 0),
      ("STEER_ANGLE", "STEERING_SENSORS", 0),
      ("STEER_ANGLE_RATE", "STEERING_SENSORS", 0),
#      ("MOTOR_TORQUE", "STEER_MOTOR_TORQUE", 0),
#      ("STEER_TORQUE_SENSOR", "STEER_STATUS", 0),
      ("LEFT_BLINKER", "LIGHT2", 0),
      ("RIGHT_BLINKER", "LIGHT2", 0),
#      ("GEAR", "GEARBOX", 0),
    #  ("SEATBELT_DRIVER_LAMP", "SEATBELT_STATUS", 1),
      ("SEATBELT_DRIVER_LATCHED", "DOORS_STATUS", 0),
#      ("BRAKE_PRESSED", "POWERTRAIN_DATA", 0),
#      ("BRAKE_SWITCH", "POWERTRAIN_DATA", 0),
      ("SET_ME", "CRUISE", 0),
  #    ("ESP_DISABLED", "VSA_STATUS", 1),
#      ("USER_BRAKE", "VSA_STATUS", 0),
#      ("BRAKE_HOLD_ACTIVE", "VSA_STATUS", 0),
#      ("STEER_STATUS", "STEER_STATUS", 5),
      ("GEAR_SHIFTER", "GEARBOX", 0),
      ("GAS_P1", "GAS_POSITION", 0),
#      ("PEDAL_GAS", "POWERTRAIN_DATA", 0),
#      ("CRUISE_SETTING", "SCM_BUTTONS", 0),
#      ("ACC_STATUS", "POWERTRAIN_DATA", 0),
  ]

  checks = [
      ("ENGINE_DATA", 100),
      ("WHEEL_SPEEDS", 50),
      ("STEERING_SENSORS", 100),
      ("DOORS_STATUS", 10),
      ("CRUISE", 10),
    #  ("POWERTRAIN_DATA", 100),
    #  ("VSA_STATUS", 50),
  ]



  signals += [("DOOR_OPEN_FL", "DOORS_STATUS", 1),
                ("DOOR_OPEN_FR", "DOORS_STATUS", 1),
                ("DOOR_OPEN_RL", "DOORS_STATUS", 1),
                ("DOOR_OPEN_RR", "DOORS_STATUS", 1),
                ("BRAKE_PRESSED", "DOORS_STATUS", 1)]
  checks += [("DOORS_STATUS", 3)]


  return signals, checks


class CarState(CarStateBase):
  def __init__(self, CP):
    super().__init__(CP)
    can_define = CANDefine(DBC[CP.carFingerprint]['pt'])
    self.shifter_values = can_define.dv["GEARBOX"]["GEAR_SHIFTER"]
  #  self.steer_status_values = defaultdict(lambda: "UNKNOWN", can_define.dv["STEER_STATUS"]["STEER_STATUS"])

    self.user_gas, self.user_gas_pressed = 0., 0
    self.brake_switch_prev = 0
    self.brake_switch_ts = 0
    self.cruise_setting = 0
    self.v_cruise_pcm_prev = 0
    self.cruise_mode = 0

    #dp
    self.lkMode = True

  def update(self, cp, cp_cam, cp_body):
    ret = car.CarState.new_message()

    # car params
    v_weight_v = [0., 1.]  # don't trust smooth speed at low values to avoid premature zero snapping
    v_weight_bp = [1., 6.]   # smooth blending, below ~0.6m/s the smooth speed snaps to zero

    # update prevs, update must run once per loop
    self.prev_cruise_buttons = self.cruise_buttons
    self.prev_cruise_setting = self.cruise_setting

    # ******************* parse out can *******************
    # TODO: find wheels moving bit in dbc
    ret.standstill = False
    ret.doorOpen = any([cp.vl["DOORS_STATUS"]['DOOR_OPEN_FL'], cp.vl["DOORS_STATUS"]['DOOR_OPEN_FR'],
                          cp.vl["DOORS_STATUS"]['DOOR_OPEN_RL'], cp.vl["DOORS_STATUS"]['DOOR_OPEN_RR']])
    ret.seatbeltUnlatched = bool(not cp.vl["DOORS_STATUS"]['SEATBELT_DRIVER_LATCHED'])

   # steer_status = self.steer_status_values[cp.vl["STEER_STATUS"]['STEER_STATUS']]
    steer_status = 'NORMAL'
    ret.steerError = steer_status not in ['NORMAL', 'NO_TORQUE_ALERT_1', 'NO_TORQUE_ALERT_2', 'LOW_SPEED_LOCKOUT', 'TMP_FAULT']
    # NO_TORQUE_ALERT_2 can be caused by bump OR steering nudge from driver
    self.steer_not_allowed = steer_status not in ['NORMAL', 'NO_TORQUE_ALERT_2']
    # LOW_SPEED_LOCKOUT is not worth a warning
    ret.steerWarning = steer_status not in ['NORMAL', 'LOW_SPEED_LOCKOUT', 'NO_TORQUE_ALERT_2']


    speed_factor = SPEED_FACTOR[self.CP.carFingerprint]
    ret.wheelSpeeds.fl = cp.vl["WHEEL_SPEEDS"]['WHEEL_SPEED_FL'] * CV.KPH_TO_MS * speed_factor
    ret.wheelSpeeds.fr = cp.vl["WHEEL_SPEEDS"]['WHEEL_SPEED_FR'] * CV.KPH_TO_MS * speed_factor
    ret.wheelSpeeds.rl = cp.vl["WHEEL_SPEEDS"]['WHEEL_SPEED_RL'] * CV.KPH_TO_MS * speed_factor
    ret.wheelSpeeds.rr = cp.vl["WHEEL_SPEEDS"]['WHEEL_SPEED_RR'] * CV.KPH_TO_MS * speed_factor
    v_wheel = (ret.wheelSpeeds.fl + ret.wheelSpeeds.fr + ret.wheelSpeeds.rl + ret.wheelSpeeds.rr)/4.

    # blend in transmission speed at low speed, since it has more low speed accuracy
    v_weight = interp(v_wheel, v_weight_bp, v_weight_v)
    ret.vEgoRaw = (1. - v_weight) * cp.vl["ENGINE_DATA"]['XMISSION_SPEED'] * CV.KPH_TO_MS * speed_factor + v_weight * v_wheel
    ret.vEgo, ret.aEgo = self.update_speed_kf(ret.vEgoRaw)

    ret.steeringAngle = cp.vl["STEERING_SENSORS"]['STEER_ANGLE']
    ret.steeringRate = cp.vl["STEERING_SENSORS"]['STEER_ANGLE_RATE']

    # dp - when user presses LKAS button on steering wheel
    if self.cruise_setting == 1:
      if cp.vl["CRUISE"]["SET_ME"] == 0:
        if self.lkMode:
          self.lkMode = False
        else:
          self.lkMode = True

    self.cruise_setting = cp.vl["CRUISE"]['SET_ME']
    self.cruise_buttons = cp.vl["CRUISE"]['SET_ME']

    self.brake_switch = cp.vl["DOORS_STATUS"]['BRAKE_PRESSED'] != 0


    ret.leftBlinker = cp.vl["LIGHT2"]['LEFT_BLINKER'] != 0
    ret.rightBlinker = cp.vl["LIGHT2"]['RIGHT_BLINKER'] != 0
#    self.brake_hold = cp.vl["VSA_STATUS"]['BRAKE_HOLD_ACTIVE']


    gear = int(cp.vl["GEARBOX"]['GEAR_SHIFTER'])
    ret.gearShifter = self.parse_gear_shifter(self.shifter_values.get(gear, None))

    ret.gas = cp.vl["GAS_POSITION"]['GAS_P1'] / 256.

    # this is a hack for the interceptor. This is now only used in the simulation
    # TODO: Replace tests by toyota so this can go away




    return ret

  @staticmethod
  def get_can_parser(CP):
    signals, checks = get_can_signals(CP)
    #bus_pt = 1 if CP.isPandaBlack and CP.carFingerprint in HONDA_BOSCH else 0
    bus_pt = 1 
    return CANParser(DBC[CP.carFingerprint]['pt'], signals, checks, bus_pt)

  @staticmethod
  def get_cam_can_parser(CP):
    signals = []
    checks = []

    # all hondas except CRV, RDX and 2019 Odyssey@China use 0xe4 for steering

   # bus_cam = 1 if CP.carFingerprint in HONDA_BOSCH and not CP.isPandaBlack else 2
    bus_cam = 1 
    return CANParser(DBC[CP.carFingerprint]['pt'], signals, checks, bus_cam)

  @staticmethod
  def get_body_can_parser(CP):
    signals = []
    checks = []


    return None
