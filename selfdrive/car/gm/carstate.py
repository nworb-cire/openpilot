from cereal import car
from common.conversions import Conversions as CV
from common.numpy_fast import mean
from opendbc.can.can_define import CANDefine
from opendbc.can.parser import CANParser
from selfdrive.car.interfaces import CarStateBase
from selfdrive.car.gm.values import CC_ONLY_CAR, DBC, AccState, CanBus, STEER_THRESHOLD

TransmissionType = car.CarParams.TransmissionType
NetworkLocation = car.CarParams.NetworkLocation


class CarState(CarStateBase):
  def __init__(self, CP):
    super().__init__(CP)
    can_define = CANDefine(DBC[CP.carFingerprint]["pt"])
    self.shifter_values = can_define.dv["ECMPRDNL2"]["PRNDL2"]
    self.loopback_lka_steering_cmd_updated = False
    self.camera_lka_steering_cmd_counter = 0
    self.buttons_counter = 0

  def update(self, pt_cp, cam_cp, loopback_cp):
    ret = car.CarState.new_message()

    self.prev_cruise_buttons = self.cruise_buttons
    self.cruise_buttons = pt_cp.vl["ASCMSteeringButton"]["ACCButtons"]
    self.buttons_counter = pt_cp.vl["ASCMSteeringButton"]["RollingCounter"]

    # Variables used for avoiding LKAS faults
    self.loopback_lka_steering_cmd_updated = len(loopback_cp.vl_all["ASCMLKASteeringCmd"]) > 0
    if self.CP.networkLocation == NetworkLocation.fwdCamera:
      self.camera_lka_steering_cmd_counter = cam_cp.vl["ASCMLKASteeringCmd"]["RollingCounter"]

    ret.wheelSpeeds = self.get_wheel_speeds(
      pt_cp.vl["EBCMWheelSpdFront"]["FLWheelSpd"],
      pt_cp.vl["EBCMWheelSpdFront"]["FRWheelSpd"],
      pt_cp.vl["EBCMWheelSpdRear"]["RLWheelSpd"],
      pt_cp.vl["EBCMWheelSpdRear"]["RRWheelSpd"],
    )
    ret.vEgoRaw = mean([ret.wheelSpeeds.fl, ret.wheelSpeeds.fr, ret.wheelSpeeds.rl, ret.wheelSpeeds.rr])
    ret.vEgo, ret.aEgo = self.update_speed_kf(ret.vEgoRaw)
    ret.standstill = ret.vEgoRaw < 0.01

    if pt_cp.vl["ECMPRDNL2"]["ManualMode"] == 1:
      ret.gearShifter = self.parse_gear_shifter("T")
    else:
      ret.gearShifter = self.parse_gear_shifter(self.shifter_values.get(pt_cp.vl["ECMPRDNL2"]["PRNDL2"], None))

    # Some Volt 2016-17 have loose brake pedal push rod retainers which causes the ECM to believe
    # that the brake is being intermittently pressed without user interaction.
    # To avoid a cruise fault we need to match the ECM's brake pressed signal and threshold
    # https://static.nhtsa.gov/odi/tsbs/2017/MC-10137629-9999.pdf
    ret.brake = pt_cp.vl["ECMAcceleratorPos"]["BrakePedalPos"]
    if self.CP.networkLocation != NetworkLocation.fwdCamera:
      ret.brakePressed = ret.brake >= 8
    else:
      # While car is braking, cancel button causes ECM to enter a soft disable state with a fault status.
      # Match ECM threshold at a standstill to allow the camera to cancel earlier
      ret.brakePressed = ret.brake >= 20

    # Regen braking is braking
    if self.CP.transmissionType == TransmissionType.direct:
      ret.brakePressed = ret.brakePressed or pt_cp.vl["EBCMRegenPaddle"]["RegenPaddle"] != 0

    if self.CP.enableGasInterceptor:
      ret.gas = (pt_cp.vl["GAS_SENSOR"]["INTERCEPTOR_GAS"] + pt_cp.vl["GAS_SENSOR"]["INTERCEPTOR_GAS2"]) / 2.
      ret.gasPressed = ret.gas > 15
    else:
      ret.gas = pt_cp.vl["AcceleratorPedal2"]["AcceleratorPedal2"] / 254.
      ret.gasPressed = ret.gas > 1e-5

    ret.steeringAngleDeg = pt_cp.vl["PSCMSteeringAngle"]["SteeringWheelAngle"]
    ret.steeringRateDeg = pt_cp.vl["PSCMSteeringAngle"]["SteeringWheelRate"]
    ret.steeringTorque = pt_cp.vl["PSCMStatus"]["LKADriverAppldTrq"]
    ret.steeringTorqueEps = pt_cp.vl["PSCMStatus"]["LKATorqueDelivered"]
    ret.steeringPressed = abs(ret.steeringTorque) > STEER_THRESHOLD

    # 0 inactive, 1 active, 2 temporarily limited, 3 failed
    self.lkas_status = pt_cp.vl["PSCMStatus"]["LKATorqueDeliveredStatus"]
    ret.steerFaultTemporary = self.lkas_status == 2
    ret.steerFaultPermanent = self.lkas_status == 3

    # 1 - open, 0 - closed
    ret.doorOpen = (pt_cp.vl["BCMDoorBeltStatus"]["FrontLeftDoor"] == 1 or
                    pt_cp.vl["BCMDoorBeltStatus"]["FrontRightDoor"] == 1 or
                    pt_cp.vl["BCMDoorBeltStatus"]["RearLeftDoor"] == 1 or
                    pt_cp.vl["BCMDoorBeltStatus"]["RearRightDoor"] == 1)

    # 1 - latched
    ret.seatbeltUnlatched = pt_cp.vl["BCMDoorBeltStatus"]["LeftSeatBelt"] == 0
    ret.leftBlinker = pt_cp.vl["BCMTurnSignals"]["TurnSignals"] == 1
    ret.rightBlinker = pt_cp.vl["BCMTurnSignals"]["TurnSignals"] == 2

    ret.parkingBrake = pt_cp.vl["VehicleIgnitionAlt"]["ParkBrake"] == 1
    ret.cruiseState.available = pt_cp.vl["ECMEngineStatus"]["CruiseMainOn"] != 0
    if self.CP.enableGasInterceptor: # Flip CC main logic when pedal is being used for long TODO: switch to cancel cc
      ret.cruiseState.available = (not ret.cruiseState.available)
    ret.espDisabled = pt_cp.vl["ESPStatus"]["TractionControlOn"] != 1

    if self.CP.carFingerprint in CC_ONLY_CAR:
      # TODO: Seek out CC state (may need to force fault...). CC may be harder to fault...
      ret.accFaulted = False # CC-only cars seem to always have value of AccState.FAULTED (3)
      ret.cruiseState.enabled = pt_cp.vl["ECMEngineStatus"]["CruiseActive"] == 1
      ret.cruiseState.standstill = False # Not possible with CC?
      ret.cruiseState.nonAdaptive = True
    else:
      ret.accFaulted = pt_cp.vl["AcceleratorPedal2"]["CruiseState"] == AccState.FAULTED # Always 3 with CC
      ret.cruiseState.enabled = pt_cp.vl["AcceleratorPedal2"]["CruiseState"] != AccState.OFF
      ret.cruiseState.standstill = pt_cp.vl["AcceleratorPedal2"]["CruiseState"] == AccState.STANDSTILL


    if self.CP.networkLocation == NetworkLocation.fwdCamera:
      ret.cruiseState.speed = cam_cp.vl["ASCMActiveCruiseControlStatus"]["ACCSpeedSetpoint"] * CV.KPH_TO_MS
      ret.stockAeb = cam_cp.vl["AEBCmd"]["AEBCmdActive"] != 0

      if self.CP.carFingerprint in CC_ONLY_CAR:
        ret.cruiseState.speed = (pt_cp.vl["ECMCruiseControl"]["CruiseSetSpeed"]) * CV.KPH_TO_MS
      else:
        ret.cruiseState.speed = (cam_cp.vl["ASCMActiveCruiseControlStatus"]["ACCSpeedSetpoint"] / 16) * CV.KPH_TO_MS
        ret.stockFcw = cam_cp.vl["ASCMActiveCruiseControlStatus"]["FCWAlert"] != 0

    return ret

  @staticmethod
  def get_cam_can_parser(CP):
    signals = []
    checks = []
    if CP.networkLocation == NetworkLocation.fwdCamera:
      signals += [
        ("AEBCmdActive", "AEBCmd"),
        ("RollingCounter", "ASCMLKASteeringCmd"),
      ]
      checks += [
        ("AEBCmd", 10),
        ("ASCMLKASteeringCmd", 10),
      ]

    if CP.networkLocation == NetworkLocation.fwdCamera and CP.carFingerprint not in CC_ONLY_CAR:
      signals.append(("FCWAlert", "ASCMActiveCruiseControlStatus"))
      signals.append(("ACCSpeedSetpoint", "ASCMActiveCruiseControlStatus"))
      checks.append(("ASCMActiveCruiseControlStatus", 25))


    return CANParser(DBC[CP.carFingerprint]["pt"], signals, checks, CanBus.CAMERA)

  @staticmethod
  def get_can_parser(CP):
    signals = [
      # sig_name, sig_address
      ("BrakePedalPos", "ECMAcceleratorPos"),
      ("FrontLeftDoor", "BCMDoorBeltStatus"),
      ("FrontRightDoor", "BCMDoorBeltStatus"),
      ("RearLeftDoor", "BCMDoorBeltStatus"),
      ("RearRightDoor", "BCMDoorBeltStatus"),
      ("LeftSeatBelt", "BCMDoorBeltStatus"),
      ("RightSeatBelt", "BCMDoorBeltStatus"),
      ("TurnSignals", "BCMTurnSignals"),
      ("AcceleratorPedal2", "AcceleratorPedal2"),
      ("CruiseState", "AcceleratorPedal2"),
      ("ACCButtons", "ASCMSteeringButton"),
      ("RollingCounter", "ASCMSteeringButton"),
      ("SteeringWheelAngle", "PSCMSteeringAngle"),
      ("SteeringWheelRate", "PSCMSteeringAngle"),
      ("FLWheelSpd", "EBCMWheelSpdFront"),
      ("FRWheelSpd", "EBCMWheelSpdFront"),
      ("RLWheelSpd", "EBCMWheelSpdRear"),
      ("RRWheelSpd", "EBCMWheelSpdRear"),
      ("PRNDL2", "ECMPRDNL2"),
      ("ManualMode", "ECMPRDNL2"),
      ("LKADriverAppldTrq", "PSCMStatus"),
      ("LKATorqueDelivered", "PSCMStatus"),
      ("LKATorqueDeliveredStatus", "PSCMStatus"),
      ("TractionControlOn", "ESPStatus"),
      ("ParkBrake", "VehicleIgnitionAlt"),
      ("CruiseMainOn", "ECMEngineStatus"),
      ("CruiseActive", "ECMEngineStatus"),
    ]

    checks = [
      ("BCMTurnSignals", 1),
      ("ECMPRDNL2", 10),
      ("PSCMStatus", 10),
      ("ESPStatus", 10),
      ("BCMDoorBeltStatus", 10),
      ("VehicleIgnitionAlt", 10),
      ("EBCMWheelSpdFront", 20),
      ("EBCMWheelSpdRear", 20),
      ("AcceleratorPedal2", 33),
      ("ASCMSteeringButton", 33),
      ("ECMEngineStatus", 100),
      ("PSCMSteeringAngle", 100),
      ("ECMAcceleratorPos", 80),
    ]


    if CP.carFingerprint in CC_ONLY_CAR:
      signals.append(("CruiseSetSpeed", "ECMCruiseControl"))
      checks.append(("ECMCruiseControl", 10))

    if CP.transmissionType == TransmissionType.direct:
      signals.append(("RegenPaddle", "EBCMRegenPaddle"))
      checks.append(("EBCMRegenPaddle", 50))

    if CP.enableGasInterceptor:
      signals.append(("INTERCEPTOR_GAS", "GAS_SENSOR"))
      signals.append(("INTERCEPTOR_GAS2", "GAS_SENSOR"))
      checks.append(("GAS_SENSOR", 50))

    return CANParser(DBC[CP.carFingerprint]["pt"], signals, checks, CanBus.POWERTRAIN)

  @staticmethod
  def get_loopback_can_parser(CP):
    passive = CP.safetyConfigs[0].safetyModel == car.CarParams.SafetyModel.noOutput
    signals = [
      ("RollingCounter", "ASCMLKASteeringCmd"),
    ]

    checks = [
      # TODO: Test this to ensure Dashcam mode works
      # TODO: This check causes false startup errors and console spamming...
      ("ASCMLKASteeringCmd", 0 if passive else 10), # 10 Hz is the stock inactive rate (every 100ms).
      #                             While active 50 Hz (every 20 ms) is normal
      #                             EPS will tolerate around 200ms when active before faulting
    ]
    return CANParser(DBC[CP.carFingerprint]["pt"], signals, checks, CanBus.LOOPBACK, passive)
