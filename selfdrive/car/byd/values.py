# flake8: noqa

from cereal import car
from selfdrive.car import dbc_dict

Ecu = car.CarParams.Ecu
VisualAlert = car.CarControl.HUDControl.VisualAlert

# Car button codes
class CruiseButtons:
  RES_ACCEL = 4
  DECEL_SET = 3
  CANCEL = 2
  MAIN = 1

# See dbc files for info on values"
VISUAL_HUD = {
  VisualAlert.none: 0,
  VisualAlert.fcw: 1,
  VisualAlert.steerRequired: 1,
  VisualAlert.brakePressed: 10,
  VisualAlert.wrongGear: 6,
  VisualAlert.seatbeltUnbuckled: 5,
  VisualAlert.speedTooHigh: 8}

class CAR:
  BYD_TANG = "BYD TANG 2018 DM"
  BYD_QIN = "BYD QIN 2014"


# diag message that in some Nidec cars only appear with 1s freq if VIN query is performed

FINGERPRINTS = {
  CAR.BYD_TANG: [{
    85: 8, 140: 8, 269: 8, 270: 8, 287: 5, 289: 8, 290: 8, 291: 8, 301: 8, 307: 8, 315: 8, 464: 8, 496: 8, 522: 8, 523: 8, 527: 8, 530: 8, 536: 8, 537: 8, 544: 8, 546: 8, 547: 8, 576: 8, 577: 8, 578: 8, 588: 8, 593: 8, 596: 8, 636: 8, 660: 8, 665: 8, 694: 8, 784: 8, 788: 8, 790: 8, 792: 8, 800: 8, 801: 8, 802: 8, 813: 8, 814: 8, 815: 8, 833: 8, 834: 8, 836: 8, 854: 8, 860: 8, 916: 8, 926: 8, 944: 8, 948: 8, 973: 8, 985: 8, 1037: 8, 1040: 8, 1058: 8, 1074: 8, 1104: 8, 1141: 8, 1152: 8, 1172: 8, 1178: 8, 1181: 8, 1193: 8, 1219: 8, 1224: 8, 1246: 8, 1269: 8, 1881: 8, 2016: 8, 2024: 8
  }],
  CAR.BYD_QIN: [{
    269: 8, 270: 8, 287: 5, 289: 8, 290: 8, 291: 8, 301: 8, 523: 8, 530: 8, 536: 8, 537: 8, 546: 8, 547: 8, 548: 8, 549: 8, 577: 8, 578: 8, 588: 8, 665: 8, 800: 8, 801: 8, 802: 8, 833: 8, 834: 8, 860: 8, 916: 8, 1040: 8, 1224: 8
  }]
}

# Don't use theses fingerprints for fingerprinting, they are still needed for ECU detection

# TODO: Figure out what is relevant
FW_VERSIONS = {
}

DBC = {
  CAR.BYD_TANG: dbc_dict('byd_qin_2014', None),
  CAR.BYD_QIN: dbc_dict('byd_qin_2014', None),
}

STEER_THRESHOLD = {
  CAR.BYD_TANG: 1200,
  CAR.BYD_QIN: 1200,
}

SPEED_FACTOR = {
  CAR.BYD_TANG: 1.025,
  CAR.BYD_QIN: 1.025,
}

# msgs sent for steering controller by camera module on can 0.
# those messages are mutually exclusive on CRV and non-CRV cars
ECU_FINGERPRINT = {
  #Ecu.fwdCamera: [0xE4, 0x194],   # steer torque cmd
}

