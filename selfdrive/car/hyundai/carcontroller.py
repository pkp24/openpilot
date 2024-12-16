from cereal import car
import cereal.messaging as messaging
from openpilot.common.conversions import Conversions as CV
from openpilot.common.numpy_fast import clip, interp
from openpilot.common.params import Params
from openpilot.common.realtime import DT_CTRL
from opendbc.can.packer import CANPacker
from openpilot.selfdrive.controls.lib.longcontrol import LongControl
from openpilot.selfdrive.car import apply_driver_steer_torque_limits, common_fault_avoidance, make_tester_present_msg
from openpilot.selfdrive.car.hyundai import hyundaicanfd, hyundaican
from openpilot.selfdrive.car.hyundai.hyundaicanfd import CanBus
from openpilot.selfdrive.car.hyundai.values import HyundaiFlags, Buttons, CarControllerParams, CANFD_CAR, CAR, CAMERA_SCC_CAR
from openpilot.selfdrive.car.interfaces import CarControllerBase

from openpilot.selfdrive.frogpilot.controls.lib.frogpilot_acceleration import get_max_allowed_accel

VisualAlert = car.CarControl.HUDControl.VisualAlert
LongCtrlState = car.CarControl.Actuators.LongControlState

# EPS faults if you apply torque while the steering angle is above 90 degrees for more than 1 second
# All slightly below EPS thresholds to avoid fault
MAX_ANGLE = 85
MAX_ANGLE_FRAMES = 89
MAX_ANGLE_CONSECUTIVE_FRAMES = 2


def process_hud_alert(enabled, fingerprint, hud_control):
  sys_warning = (hud_control.visualAlert in (VisualAlert.steerRequired, VisualAlert.ldw))

  # initialize to no line visible
  # TODO: this is not accurate for all cars
  sys_state = 1
  if hud_control.leftLaneVisible and hud_control.rightLaneVisible or sys_warning:  # HUD alert only display when LKAS status is active
    sys_state = 3 if enabled or sys_warning else 4
  elif hud_control.leftLaneVisible:
    sys_state = 5
  elif hud_control.rightLaneVisible:
    sys_state = 6

  # initialize to no warnings
  left_lane_warning = 0
  right_lane_warning = 0
  if hud_control.leftLaneDepart:
    left_lane_warning = 1 if fingerprint in (CAR.GENESIS_G90, CAR.GENESIS_G80) else 2
  if hud_control.rightLaneDepart:
    right_lane_warning = 1 if fingerprint in (CAR.GENESIS_G90, CAR.GENESIS_G80) else 2

  return sys_warning, sys_state, left_lane_warning, right_lane_warning


class CarController(CarControllerBase):
  def __init__(self, dbc_name, CP, VM):
    self.CP = CP
    self.CAN = CanBus(CP)
    self.params = CarControllerParams(CP)
    self.packer = CANPacker(dbc_name)
    self.angle_limit_counter = 0
    self.frame = 0
    self.jerk = 0.0
    self.accel_last = 0
    self.apply_steer_last = 0
    self.car_fingerprint = CP.carFingerprint
    self.last_button_frame = 0
    self.LoC = LongControl(CP)

    sub_services = ['longitudinalPlan']
    if CP.openpilotLongitudinalControl:
      sub_services.append('radarState')

    if sub_services:
      self.sm = messaging.SubMaster(sub_services)

    self.param_s = Params()
    self.hkg_tuning = self.param_s.get_bool('HKGtuning')
    self.jerk_limiter = JerkLimiter(DT_CTRL)


  def update(self, CC, CS, now_nanos, frogpilot_toggles):
    actuators = CC.actuators
    hud_control = CC.hudControl
    accel = actuators.accel

    #Update HKG tuning state
    self.hkg_tuning = frogpilot_toggles.hkg_tuning

    # steering torque
    new_steer = int(round(actuators.steer * self.params.STEER_MAX))
    apply_steer = apply_driver_steer_torque_limits(new_steer, self.apply_steer_last, CS.out.steeringTorque, self.params)


    # >90 degree steering fault prevention
    self.angle_limit_counter, apply_steer_req = common_fault_avoidance(abs(CS.out.steeringAngleDeg) >= MAX_ANGLE, CC.latActive,
                                                                       self.angle_limit_counter, MAX_ANGLE_FRAMES,
                                                                       MAX_ANGLE_CONSECUTIVE_FRAMES)

    if not CC.latActive:
      apply_steer = 0

    # Hold torque with induced temporary fault when cutting the actuation bit
    torque_fault = CC.latActive and not apply_steer_req

    self.apply_steer_last = apply_steer

    # accel + longitudinal
    if self.hkg_tuning and self.CP.openpilotLongitudinalControl:
      accel = self.jerk_limiter.calculate_limited_accel(
                    accel, actuators, CS, LongCtrlState, interp, clip)

    if frogpilot_toggles.sport_plus:
      accel = clip(actuators.accel, CarControllerParams.ACCEL_MIN, min(frogpilot_toggles.max_desired_acceleration, get_max_allowed_accel(CS.out.vEgo)))

    else:
      accel = clip(actuators.accel, CarControllerParams.ACCEL_MIN, min(frogpilot_toggles.max_desired_acceleration, CarControllerParams.ACCEL_MAX))
    stopping = actuators.longControlState == LongCtrlState.stopping
    set_speed_in_units = hud_control.setSpeed * (CV.MS_TO_KPH if CS.is_metric else CV.MS_TO_MPH)

    # HUD messages
    sys_warning, sys_state, left_lane_warning, right_lane_warning = process_hud_alert(CC.enabled, self.car_fingerprint,
                                                                                      hud_control)

    can_sends = []

    # *** common hyundai stuff ***

    # tester present - w/ no response (keeps relevant ECU disabled)
    if self.frame % 100 == 0 and not (self.CP.flags & HyundaiFlags.CANFD_CAMERA_SCC.value) and \
      self.CP.carFingerprint not in CAMERA_SCC_CAR and self.CP.openpilotLongitudinalControl:
      # for longitudinal control, either radar or ADAS driving ECU
      addr, bus = 0x7d0, self.CAN.ECAN if self.CP.carFingerprint in CANFD_CAR else 0
      if self.CP.flags & HyundaiFlags.CANFD_HDA2.value:
        addr, bus = 0x730, self.CAN.ECAN
      can_sends.append(make_tester_present_msg(addr, bus, suppress_response=True))

      # for blinkers
      if self.CP.flags & HyundaiFlags.ENABLE_BLINKERS:
        can_sends.append(make_tester_present_msg(0x7b1, self.CAN.ECAN, suppress_response=True))

    # CAN-FD platforms
    if self.CP.carFingerprint in CANFD_CAR:
      hda2 = self.CP.flags & HyundaiFlags.CANFD_HDA2
      hda2_long = hda2 and self.CP.openpilotLongitudinalControl

      # steering control
      can_sends.extend(hyundaicanfd.create_steering_messages(self.packer, self.CP, self.CAN, CC.enabled, apply_steer_req, apply_steer))

      # prevent LFA from activating on HDA2 by sending "no lane lines detected" to ADAS ECU
      if self.frame % 5 == 0 and hda2:
        can_sends.append(hyundaicanfd.create_suppress_lfa(self.packer, self.CAN, CS.hda2_lfa_block_msg,
                                                          self.CP.flags & HyundaiFlags.CANFD_HDA2_ALT_STEERING))

      # LFA and HDA icons
      if self.frame % 5 == 0 and (not hda2 or hda2_long):
        can_sends.append(hyundaicanfd.create_lfahda_cluster(self.packer, self.CAN, CC.enabled, CC.latActive))

      # blinkers
      if hda2 and self.CP.flags & HyundaiFlags.ENABLE_BLINKERS:
        can_sends.extend(hyundaicanfd.create_spas_messages(self.packer, self.CAN, self.frame, CC.leftBlinker, CC.rightBlinker))

      if self.CP.openpilotLongitudinalControl:
        if hda2:
          can_sends.extend(hyundaicanfd.create_adrv_messages(self.packer, self.CAN, self.frame))
        else:
          can_sends.extend(hyundaicanfd.create_fca_warning_light(self.packer, self.CAN, self.frame))
        if self.frame % 2 == 0:
          can_sends.append(hyundaicanfd.create_acc_control(self.packer, self.CAN, CS, CC.enabled, self.accel_last, accel,
                                                           stopping, CC.cruiseControl.override, set_speed_in_units, hud_control))
          self.accel_last = accel
      else:
        # button presses
        can_sends.extend(self.create_button_messages(CC, CS, use_clu11=False))
    else:
      can_sends.append(hyundaican.create_lkas11(self.packer, self.frame, self.CP, apply_steer, apply_steer_req,
                                                torque_fault, CS.lkas11, sys_warning, sys_state, CC.enabled,
                                                hud_control.leftLaneVisible, hud_control.rightLaneVisible,
                                                left_lane_warning, right_lane_warning))

      if not self.CP.openpilotLongitudinalControl:
        can_sends.extend(self.create_button_messages(CC, CS, use_clu11=True))


      if self.frame % 2 == 0 and self.CP.openpilotLongitudinalControl:
        # TODO: unclear if this is needed
        jerk = 3.0 if actuators.longControlState == LongCtrlState.pid else 1.0
        use_fca = self.CP.flags & HyundaiFlags.USE_FCA.value
        can_sends.extend(hyundaican.create_acc_commands(self.packer, CC.enabled, accel, jerk, int(self.frame / 2),
                                                        hud_control, set_speed_in_units, stopping,
                                                        CC.cruiseControl.override, use_fca, CS, self.CP, CS.out.cruiseState.available))

      # 20 Hz LFA MFA message
      if self.frame % 5 == 0 and self.CP.flags & HyundaiFlags.SEND_LFA.value:
        can_sends.append(hyundaican.create_lfahda_mfc(self.packer, CC.enabled, CC.latActive))

      # 5 Hz ACC options
      if self.frame % 20 == 0 and self.CP.openpilotLongitudinalControl:
        can_sends.extend(hyundaican.create_acc_opt(self.packer, CS, self.CP))

      # 2 Hz front radar options
      if self.frame % 50 == 0 and self.CP.openpilotLongitudinalControl:
        can_sends.append(hyundaican.create_frt_radar_opt(self.packer))

    new_actuators = actuators.as_builder()
    new_actuators.steer = apply_steer / self.params.STEER_MAX
    new_actuators.steerOutputCan = apply_steer
    new_actuators.accel = accel

    self.frame += 1
    return new_actuators, can_sends

  def create_button_messages(self, CC: car.CarControl, CS: car.CarState, use_clu11: bool):
    can_sends = []
    if use_clu11:
      if CC.cruiseControl.cancel:
        can_sends.append(hyundaican.create_clu11(self.packer, self.frame, CS.clu11, Buttons.CANCEL, self.CP))
      elif CC.cruiseControl.resume:
        # send resume at a max freq of 10Hz
        if (self.frame - self.last_button_frame) * DT_CTRL > 0.1:
          # send 25 messages at a time to increases the likelihood of resume being accepted
          can_sends.extend([hyundaican.create_clu11(self.packer, self.frame, CS.clu11, Buttons.RES_ACCEL, self.CP)] * 25)
          if (self.frame - self.last_button_frame) * DT_CTRL >= 0.15:
            self.last_button_frame = self.frame
    else:
      if (self.frame - self.last_button_frame) * DT_CTRL > 0.25:
        # cruise cancel
        if CC.cruiseControl.cancel:
          if self.CP.flags & HyundaiFlags.CANFD_ALT_BUTTONS:
            can_sends.append(hyundaicanfd.create_acc_cancel(self.packer, self.CP, self.CAN, CS.cruise_info))
            self.last_button_frame = self.frame
          else:
            for _ in range(20):
              can_sends.append(hyundaicanfd.create_buttons(self.packer, self.CP, self.CAN, CS.buttons_counter+1, Buttons.CANCEL))
            self.last_button_frame = self.frame

        # cruise standstill resume
        elif CC.cruiseControl.resume:
          if self.CP.flags & HyundaiFlags.CANFD_ALT_BUTTONS:
            # TODO: resume for alt button cars
            pass
          else:
            for _ in range(20):
              can_sends.append(hyundaicanfd.create_buttons(self.packer, self.CP, self.CAN, CS.buttons_counter+1, Buttons.RES_ACCEL))
            self.last_button_frame = self.frame

    return can_sends

class JerkLimiter:
  def __init__(self, DT_CTRL):
      self.jerk_count = 0.0
      self.jerk_upper_limit = 0.0
      self.jerk_lower_limit = 0.0
      self.accel_last = 0
      self.accel_last_jerk = 0
      self.accel_raw = 0
      self.DT_CTRL = DT_CTRL
      self.jerk = 0
      self.param_s = Params()
      self.hkg_tuning = self.param_s.get_bool('HKGtuning')

  def cal_jerk(self, accel, actuators):
    self.accel_raw = accel

    if actuators.longControlState == LongCtrlState.off:
        accel_diff = 0.0
    elif self.accel_last_jerk < 0 and accel > self.accel_last_jerk:
        accel_diff = (self.accel_raw - self.accel_last_jerk) * 0.7
    elif actuators.longControlState == LongCtrlState.stopping:
        accel_diff = 0.0
    else:
        accel_diff = self.accel_raw - self.accel_last_jerk

    accel_diff /= self.DT_CTRL
    self.jerk = self.jerk * 0.95 + accel_diff * 0.05
    self.accel_last_jerk = self.accel_raw
    return self.jerk

  def make_jerk(self, CS, accel, actuators):
    jerk = self.cal_jerk(accel, actuators)
    a_error = accel - CS.out.aEgo
    jerk = jerk + (a_error * 2.0)

    if not self.hkg_tuning:
        self.jerk_upper_limit = 3 if actuators.longControlState == LongCtrlState.pid else 1.0
        self.jerk_lower_limit = 3
    else:
        startingJerk = 0.5
        jerkLimit = 3.0
        self.jerk_count += self.DT_CTRL
        jerk_max = interp(self.jerk_count, [0, 1.5, 2.5], [startingJerk, startingJerk, jerkLimit])

        if actuators.longControlState == LongCtrlState.off:
            self.jerk_upper_limit = jerkLimit
            self.jerk_lower_limit = jerkLimit
            self.jerk_count = 0
        elif accel < 0:
            self.jerk_upper_limit *= jerkLimit
            self.jerk_lower_limit *= 0.9
        else:
            self.jerk_upper_limit = min(max(0.5, jerk * 2.0), jerk_max)
            self.jerk_lower_limit = min(max(0.9, -jerk * 2.0), jerkLimit)

    return jerk


  def calculate_limited_accel(self, accel, actuators, CS, LongCtrlState, interp, clip):
    """Limit acceleration based on jerk limits"""

    self.make_jerk(CS, accel, actuators)
    accel_delta = accel - self.accel_last

    if accel < 0:
      abs_accel = clip(abs(accel), 0.0, 2.0)
      damp_factor = interp(abs_accel, [0, 2.0], [0.8, 0.6])
      accel_delta *= damp_factor

    if abs(accel_delta) > 0.5:  # For large mph changes
      accel_delta *= 0.8  # Additional damping

    limited_accel_delta = clip(accel_delta, -self.jerk_lower_limit * self.DT_CTRL,
                               self.jerk_upper_limit * self.DT_CTRL)

    if accel < 0 and self.accel_last > 0:
      limited_accel_delta *= 0.7  # Softer transition to braking

    accel = self.accel_last + limited_accel_delta
    self.accel_last = accel

    return accel

