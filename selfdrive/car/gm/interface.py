#!/usr/bin/env python3
from cereal import car
from math import fabs
from panda import Panda

from common.conversions import Conversions as CV
from selfdrive.car import STD_CARGO_KG, create_button_event, scale_rot_inertia, scale_tire_stiffness, gen_empty_fingerprint, get_safety_config
from selfdrive.car.gm.values import CAR, CC_ONLY_CAR, CruiseButtons, CarControllerParams, EV_CAR, CAMERA_ACC_CAR
from selfdrive.car.interfaces import CarInterfaceBase

ButtonType = car.CarState.ButtonEvent.Type
EventName = car.CarEvent.EventName
GearShifter = car.CarState.GearShifter
TransmissionType = car.CarParams.TransmissionType
NetworkLocation = car.CarParams.NetworkLocation
BUTTONS_DICT = {CruiseButtons.RES_ACCEL: ButtonType.accelCruise, CruiseButtons.DECEL_SET: ButtonType.decelCruise,
                CruiseButtons.MAIN: ButtonType.altButton3, CruiseButtons.CANCEL: ButtonType.cancel}


class CarInterface(CarInterfaceBase):
  @staticmethod
  def get_pid_accel_limits(CP, current_speed, cruise_speed):
    params = CarControllerParams()
    return params.ACCEL_MIN, params.ACCEL_MAX

  # Determined by iteratively plotting and minimizing error for f(angle, speed) = steer.
  @staticmethod
  def get_steer_feedforward_volt(desired_angle, v_ego):
    desired_angle *= 0.02904609
    sigmoid = desired_angle / (1 + fabs(desired_angle))
    return 0.10006696 * sigmoid * (v_ego + 3.12485927)

  @staticmethod
  def get_steer_feedforward_volt_torque(desired_lateral_accel, v_ego):
    ANGLE_COEF = 0.10000000
    ANGLE_COEF2 = 0.13609369
    ANGLE_OFFSET = 0.00215830
    SPEED_OFFSET = -3.40740140
    SIGMOID_COEF_RIGHT = 0.54324908
    SIGMOID_COEF_LEFT = 0.48109872
    SPEED_COEF = 0.59589789
    return get_steer_feedforward_erf(desired_lateral_accel, v_ego, ANGLE_COEF, ANGLE_COEF2, ANGLE_OFFSET, SPEED_OFFSET, SIGMOID_COEF_RIGHT, SIGMOID_COEF_LEFT, SPEED_COEF)

  @staticmethod
  def get_steer_feedforward_acadia(desired_angle, v_ego):
    desired_angle *= 0.09760208
    sigmoid = desired_angle / (1 + fabs(desired_angle))
    return 0.04689655 * sigmoid * (v_ego + 10.028217)

  def get_steer_feedforward_function(self):
    if self.CP.carFingerprint == CAR.VOLT or self.CP.carFingerprint == CAR.VOLT_NR:
      return self.get_steer_feedforward_volt_torque
    elif self.CP.carFingerprint == CAR.ACADIA:
      return self.get_steer_feedforward_acadia
    else:
      return CarInterfaceBase.get_steer_feedforward_default

  @staticmethod
  def get_params(candidate, fingerprint=gen_empty_fingerprint(), car_fw=None, experimental_long=False):
    ret = CarInterfaceBase.get_std_params(candidate, fingerprint)
    ret.carName = "gm"
    ret.safetyConfigs = [get_safety_config(car.CarParams.SafetyModel.gm)]
    ret.autoResumeSng = False

    if candidate in EV_CAR:
      ret.transmissionType = TransmissionType.direct
    else:
      ret.transmissionType = TransmissionType.automatic

    if candidate in CAMERA_ACC_CAR:
      ret.openpilotLongitudinalControl = False
      ret.networkLocation = NetworkLocation.fwdCamera
      ret.radarOffCan = True  # no radar
      ret.pcmCruise = True
      ret.safetyConfigs[0].safetyParam |= Panda.FLAG_GM_HW_CAM

      if candidate in CC_ONLY_CAR:
        ret.safetyConfigs[0].safetyParam |= Panda.FLAG_GM_HW_CAM_CC
        # TODO: Add Toggle
        ret.openpilotLongitudinalControl = True

        ret.enableGasInterceptor = 0x201 in fingerprint[0]
        if ret.enableGasInterceptor:
          ret.openpilotLongitudinalControl = True
          ret.pcmCruise = False

    else:  # ASCM, OBD-II harness
      ret.openpilotLongitudinalControl = True
      ret.networkLocation = NetworkLocation.gateway
      ret.radarOffCan = False
      ret.pcmCruise = False  # stock non-adaptive cruise control is kept off

    # These cars have been put into dashcam only due to both a lack of users and test coverage.
    # These cars likely still work fine. Once a user confirms each car works and a test route is
    # added to selfdrive/car/tests/routes.py, we can remove it from this list.
    ret.dashcamOnly = candidate in {CAR.CADILLAC_ATS, CAR.HOLDEN_ASTRA, CAR.MALIBU, CAR.BUICK_REGAL, CAR.EQUINOX, CAR.BOLT_EV}

    # Start with a baseline tuning for all GM vehicles. Override tuning as needed in each model section below.
    # Some GMs need some tolerance above 10 kph to avoid a fault
    ret.minSteerSpeed = 10.1 * CV.KPH_TO_MS
    ret.lateralTuning.pid.kiBP, ret.lateralTuning.pid.kpBP = [[0.], [0.]]
    ret.lateralTuning.pid.kpV, ret.lateralTuning.pid.kiV = [[0.2], [0.00]]
    ret.lateralTuning.pid.kf = 0.00004   # full torque for 20 deg at 80mph means 0.00007818594
    ret.steerActuatorDelay = 0.1  # Default delay, not measured yet
    tire_stiffness_factor = 0.444  # not optimized yet

    ret.longitudinalTuning.kpBP = [5., 35.]
    ret.longitudinalTuning.kpV = [2.4, 1.5]
    ret.longitudinalTuning.kiBP = [0.]
    ret.longitudinalTuning.kiV = [0.36]

    ret.steerLimitTimer = 0.4
    ret.radarTimeStep = 0.0667  # GM radar runs at 15Hz instead of standard 20Hz

    # supports stop and go, but initial engage must (conservatively) be above 18mph
    ret.minEnableSpeed = 18 * CV.MPH_TO_MS
    if ret.enableGasInterceptor:
          ret.minEnableSpeed = -1

    if candidate == CAR.VOLT or candidate == CAR.VOLT_NR:
      ret.mass = 1607. + STD_CARGO_KG
      ret.wheelbase = 2.69
      ret.steerRatio = 17.7  # Stock 15.7, LiveParameters
      tire_stiffness_factor = 0.469  # Stock Michelin Energy Saver A/S, LiveParameters
      ret.centerToFront = ret.wheelbase * 0.45  # Volt Gen 1, TODO corner weigh

      # ret.lateralTuning.pid.kpBP = [0., 40.]
      # ret.lateralTuning.pid.kpV = [0., 0.17]
      # ret.lateralTuning.pid.kiBP = [0.]
      # ret.lateralTuning.pid.kiV = [0.]
      # ret.lateralTuning.pid.kf = 1.  # get_steer_feedforward_volt()
      ret.steerActuatorDelay = 0.18

      max_lateral_accel = 3.0
      ret.lateralTuning.init('torque')
      ret.lateralTuning.torque.useSteeringAngle = True
      ret.lateralTuning.torque.kp = 1.8 / max_lateral_accel
      ret.lateralTuning.torque.ki = 0.5 / max_lateral_accel
      ret.lateralTuning.torque.kd = 5.0 / max_lateral_accel
      ret.lateralTuning.torque.kf = 1.0 # use with custom torque ff
      ret.lateralTuning.torque.friction = 0.005

      if ret.enableGasInterceptor:
        ret.longitudinalActuatorDelayLowerBound = 0.06
        ret.longitudinalActuatorDelayUpperBound = 0.08
        #Note: Low speed, stop and go not tested. Should be fairly smooth on highway
        ret.longitudinalTuning.kpBP = [0., 35.0]
        ret.longitudinalTuning.kpV = [0.4, 0.06]
        ret.longitudinalTuning.kiBP = [0., 35.0]
        ret.longitudinalTuning.kiV = [0.0, 0.04]
        ret.longitudinalTuning.kf = 0.25
        ret.stoppingDecelRate = 0.8  # reach stopping target smoothly, brake_travel/s while trying to stop
        ret.stopAccel = 0. # Required acceleraton to keep vehicle stationary
        ret.vEgoStopping = 0.5  # Speed at which the car goes into stopping state, when car starts requesting stopping accel
        ret.vEgoStarting = 0.5  # Speed at which the car goes into starting state, when car starts requesting starting accel,
        # vEgoStarting needs to be > or == vEgoStopping to avoid state transition oscillation
        ret.stoppingControl = True

    elif candidate == CAR.MALIBU:
      ret.mass = 1496. + STD_CARGO_KG
      ret.wheelbase = 2.83
      ret.steerRatio = 15.8
      ret.centerToFront = ret.wheelbase * 0.4  # wild guess

    elif candidate == CAR.HOLDEN_ASTRA:
      ret.mass = 1363. + STD_CARGO_KG
      ret.wheelbase = 2.662
      # Remaining parameters copied from Volt for now
      ret.centerToFront = ret.wheelbase * 0.4
      ret.steerRatio = 15.7

    elif candidate == CAR.ACADIA:
      ret.minEnableSpeed = -1.  # engage speed is decided by pcm
      ret.mass = 4353. * CV.LB_TO_KG + STD_CARGO_KG
      ret.wheelbase = 2.86
      ret.steerRatio = 14.4  # end to end is 13.46
      ret.centerToFront = ret.wheelbase * 0.4
      ret.lateralTuning.pid.kf = 1.  # get_steer_feedforward_acadia()
      ret.longitudinalActuatorDelayUpperBound = 0.5  # large delay to initially start braking

    elif candidate == CAR.BUICK_REGAL:
      ret.mass = 3779. * CV.LB_TO_KG + STD_CARGO_KG  # (3849+3708)/2
      ret.wheelbase = 2.83  # 111.4 inches in meters
      ret.steerRatio = 14.4  # guess for tourx
      ret.centerToFront = ret.wheelbase * 0.4  # guess for tourx

    elif candidate == CAR.CADILLAC_ATS:
      ret.mass = 1601. + STD_CARGO_KG
      ret.wheelbase = 2.78
      ret.steerRatio = 15.3
      ret.centerToFront = ret.wheelbase * 0.5

    elif candidate == CAR.ESCALADE_ESV:
      ret.minEnableSpeed = -1.  # engage speed is decided by pcm
      ret.mass = 2739. + STD_CARGO_KG
      ret.wheelbase = 3.302
      ret.steerRatio = 17.3
      ret.centerToFront = ret.wheelbase * 0.5
      ret.lateralTuning.pid.kiBP, ret.lateralTuning.pid.kpBP = [[10., 41.0], [10., 41.0]]
      ret.lateralTuning.pid.kpV, ret.lateralTuning.pid.kiV = [[0.13, 0.24], [0.01, 0.02]]
      ret.lateralTuning.pid.kf = 0.000045
      tire_stiffness_factor = 1.0

    elif candidate in (CAR.BOLT_EV, CAR.BOLT_EUV):
      ret.minEnableSpeed = -1
      ret.mass = 1669. + STD_CARGO_KG
      ret.wheelbase = 2.63779
      ret.steerRatio = 16.8
      ret.centerToFront = 2.15  # measured
      tire_stiffness_factor = 1.0
      ret.steerActuatorDelay = 0.2
      CarInterfaceBase.configure_torque_tune(candidate, ret.lateralTuning)

    elif candidate == CAR.SILVERADO:
      ret.minEnableSpeed = -1
      ret.mass = 2200. + STD_CARGO_KG
      ret.wheelbase = 3.75
      ret.steerRatio = 16.3
      ret.centerToFront = ret.wheelbase * 0.5
      tire_stiffness_factor = 1.0
      CarInterfaceBase.configure_torque_tune(candidate, ret.lateralTuning)

    elif candidate == CAR.EQUINOX:
      ret.minEnableSpeed = -1
      ret.mass = 3500. * CV.LB_TO_KG + STD_CARGO_KG
      ret.wheelbase = 2.72
      ret.steerRatio = 14.4
      ret.centerToFront = ret.wheelbase * 0.4
      CarInterfaceBase.configure_torque_tune(candidate, ret.lateralTuning)

    elif candidate == CAR.BOLT_EV_CC:
      ret.minEnableSpeed = -1
      ret.mass = 1669. + STD_CARGO_KG
      ret.wheelbase = 2.601
      ret.steerRatio = 16.8
      ret.centerToFront = 2.0828 # Measured
      tire_stiffness_factor = 1.0
      ret.steerActuatorDelay = 0.2
      CarInterfaceBase.configure_torque_tune(candidate, ret.lateralTuning)

      if ret.enableGasInterceptor:
        #Note: Low speed, stop and go not tested. Should be fairly smooth on highway
        ret.longitudinalTuning.kpV = [0.4, 0.06]
        ret.longitudinalTuning.kiBP = [0., 35.0]
        ret.longitudinalTuning.kiV = [0.0, 0.04]
        ret.longitudinalTuning.kiV = [0.0, 0.04]
        ret.longitudinalTuning.kf = 0.3
        ret.stoppingDecelRate = 0.8  # reach stopping target smoothly, brake_travel/s while trying to stop
        ret.stopAccel = 0. # Required acceleration to keep vehicle stationary
        ret.vEgoStopping = 0.5  # Speed at which the car goes into stopping state, when car starts requesting stopping accel
        ret.vEgoStarting = 0.5  # Speed at which the car goes into starting state, when car starts requesting starting accel,
        # vEgoStarting needs to be > or == vEgoStopping to avoid state transition oscillation
        ret.stoppingControl = True

    # TODO: get actual value, for now starting with reasonable value for
    # civic and scaling by mass and wheelbase
    ret.rotationalInertia = scale_rot_inertia(ret.mass, ret.wheelbase)

    # TODO: start from empirically derived lateral slip stiffness for the civic and scale by
    # mass and CG position, so all cars will have approximately similar dyn behaviors
    ret.tireStiffnessFront, ret.tireStiffnessRear = scale_tire_stiffness(ret.mass, ret.wheelbase, ret.centerToFront,
                                                                         tire_stiffness_factor=tire_stiffness_factor)

    return ret

  # returns a car.CarState
  def _update(self, c):
    ret = self.CS.update(self.cp, self.cp_cam, self.cp_loopback)

    if self.CS.cruise_buttons != self.CS.prev_cruise_buttons and self.CS.prev_cruise_buttons != CruiseButtons.INIT:
      be = create_button_event(self.CS.cruise_buttons, self.CS.prev_cruise_buttons, BUTTONS_DICT, CruiseButtons.UNPRESS)

      # Suppress resume button if we're resuming from stop so we don't adjust speed.
      if be.type == ButtonType.accelCruise and (ret.cruiseState.enabled and ret.standstill):
        be.type = ButtonType.unknown

      ret.buttonEvents = [be]

    events = self.create_common_events(ret, extra_gears=[GearShifter.sport, GearShifter.low,
                                                         GearShifter.eco, GearShifter.manumatic],
                                       pcm_enable=self.CP.pcmCruise)

    if ret.vEgo < self.CP.minEnableSpeed:
      events.add(EventName.belowEngageSpeed)
    if ret.cruiseState.standstill:
      events.add(EventName.resumeRequired)
    if ret.vEgo < self.CP.minSteerSpeed:
      events.add(EventName.belowSteerSpeed)

    if self.CP.networkLocation == NetworkLocation.fwdCamera and self.CP.pcmCruise:
      # The ECM has a higher brake pressed threshold than the camera, causing an
      # ACC fault when you engage at a stop with your foot partially on the brake
      if ret.vEgoRaw < 0.1 and ret.brake < 20:
        events.add(EventName.gmAccFaultedTemp)

    ret.events = events.to_msg()

    return ret

  def apply(self, c):
    return self.CC.update(c, self.CS)
