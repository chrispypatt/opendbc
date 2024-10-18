from openpilot.common.params import Params
import math
from opendbc.car import carlog, apply_meas_steer_torque_limits, apply_std_steer_angle_limits, common_fault_avoidance, \
                        make_tester_present_msg, rate_limit, structs
from opendbc.car.can_definitions import CanData
from opendbc.car.common.numpy_fast import clip
from opendbc.car.secoc import add_mac, build_sync_mac
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.toyota import toyotacan
from opendbc.car.toyota.values import CAR, STATIC_DSU_MSGS, NO_STOP_TIMER_CAR, TSS2_CAR, \
                                        CarControllerParams, ToyotaFlags, \
                                        UNSUPPORTED_DSU_CAR
from opendbc.can.packer import CANPacker

LongCtrlState = structs.CarControl.Actuators.LongControlState
SteerControlType = structs.CarParams.SteerControlType
VisualAlert = structs.CarControl.HUDControl.VisualAlert
LongCtrlState = structs.CarControl.Actuators.LongControlState

ACCELERATION_DUE_TO_GRAVITY = 9.81  # m/s^2

ACCEL_WINDUP_LIMIT = 0.5  # m/s^2 / frame

# LKA limits
# EPS faults if you apply torque while the steering rate is above 100 deg/s for too long
MAX_STEER_RATE = 100  # deg/s
MAX_STEER_RATE_FRAMES = 18  # tx control frames needed before torque can be cut

# EPS allows user torque above threshold for 50 frames before permanently faulting
MAX_USER_TORQUE = 500

# Ale Sato stuff
UNLOCK_CMD =       b'\x40\x05\x30\x11\x00\x40\x00\x00'
LOCK_CMD =         b'\x40\x05\x30\x11\x00\x80\x00\x00'
HORN_ON_CMD =      b'\x40\x04\x30\x06\x00\x20\x00\x00'
HORN_OFF_CMD =     b'\x40\x04\x30\x06\x00\x00\x00\x00'
HIGHBEAM_ON_CMD =  b'\x40\x06\x30\x15\x00\x20\x00\x00'
HIGHBEAM_OFF_CMD = b'\x40\x06\x30\x15\x00\x00\x00\x00'

# LTA limits
# EPS ignores commands above this angle and causes PCS to fault
MAX_LTA_ANGLE = 94.9461  # deg
MAX_LTA_DRIVER_TORQUE_ALLOWANCE = 150  # slightly above steering pressed allows some resistance when changing lanes


class CarController(CarControllerBase):
  def __init__(self, dbc_name, CP):
    super().__init__(dbc_name, CP)
    self.params = CarControllerParams(self.CP)
    self.last_steer = 0
    self.last_angle = 0
    self.alert_active = False
    self.last_standstill = False
    self.standstill_req = False
    self.steer_rate_counter = 0
    self.distance_button = 0

    self.pcm_accel_compensation = 0.0
    self.permit_braking = True

    self.packer = CANPacker(dbc_name)
    self.accel = 0


    # AleSato stuff
    self.remoteLockDoors = False
    self.lastRemoteLockDoors = False
    self.oneHonk = False
    self.twoHonks = False
    self.honk_rate_counter = 0


    self.reset_pcm_compensation = True
    self.secoc_lka_message_counter = 0
    self.secoc_lta_message_counter = 0
    self.secoc_prev_reset_counter = 0
    self.secoc_mismatch_counter = 0

  def update(self, CC, CS, now_nanos):
    actuators = CC.actuators
    hud_control = CC.hudControl
    pcm_cancel_cmd = CC.cruiseControl.cancel
    lat_active = CC.latActive and abs(CS.out.steeringTorque) < MAX_USER_TORQUE

    # AleSato stuff
    stopping = actuators.longControlState == LongCtrlState.stopping and not CS.out.gasPressed

    # *** control msgs ***
    can_sends = []

    # *** handle secoc reset counter increase ***
    if self.CP.flags & ToyotaFlags.SECOC.value:
      if CS.secoc_synchronization['RESET_CNT'] != self.secoc_prev_reset_counter:
        self.secoc_lka_message_counter = 0
        self.secoc_lta_message_counter = 0
        self.secoc_prev_reset_counter = CS.secoc_synchronization['RESET_CNT']

        expected_mac = build_sync_mac(self.secoc_key, int(CS.secoc_synchronization['TRIP_CNT']), int(CS.secoc_synchronization['RESET_CNT']))
        if int(CS.secoc_synchronization['AUTHENTICATOR']) != expected_mac and self.secoc_mismatch_counter < 100:
          carlog.error("SecOC synchronization MAC mismatch, wrong key?")
          self.secoc_mismatch_counter += 1

    # *** steer torque ***
    new_steer = int(round(actuators.steer * self.params.STEER_MAX))
    apply_steer = apply_meas_steer_torque_limits(new_steer, self.last_steer, CS.out.steeringTorqueEps, self.params)

    # >100 degree/sec steering fault prevention
    self.steer_rate_counter, apply_steer_req = common_fault_avoidance(abs(CS.out.steeringRateDeg) >= MAX_STEER_RATE, lat_active,
                                                                      self.steer_rate_counter, MAX_STEER_RATE_FRAMES)

    if not lat_active:
      apply_steer = 0

    # *** steer angle ***
    if self.CP.steerControlType == SteerControlType.angle:
      # If using LTA control, disable LKA and set steering angle command
      apply_steer = 0
      apply_steer_req = False
      if self.frame % 2 == 0:
        # EPS uses the torque sensor angle to control with, offset to compensate
        apply_angle = actuators.steeringAngleDeg + CS.out.steeringAngleOffsetDeg

        # Angular rate limit based on speed
        apply_angle = apply_std_steer_angle_limits(apply_angle, self.last_angle, CS.out.vEgoRaw, self.params)

        if not lat_active:
          apply_angle = CS.out.steeringAngleDeg + CS.out.steeringAngleOffsetDeg

        self.last_angle = clip(apply_angle, -MAX_LTA_ANGLE, MAX_LTA_ANGLE)

    self.last_steer = apply_steer

    # toyota can trace shows STEERING_LKA at 42Hz, with counter adding alternatively 1 and 2;
    # sending it at 100Hz seem to allow a higher rate limit, as the rate limit seems imposed
    # on consecutive messages
    steer_command = toyotacan.create_steer_command(self.packer, apply_steer, apply_steer_req)
    if self.CP.flags & ToyotaFlags.SECOC.value:
      # TODO: check if this slow and needs to be done by the CANPacker
      steer_command = add_mac(self.secoc_key,
                              int(CS.secoc_synchronization['TRIP_CNT']),
                              int(CS.secoc_synchronization['RESET_CNT']),
                              self.secoc_lka_message_counter,
                              steer_command)
      self.secoc_lka_message_counter += 1
    can_sends.append(steer_command)

    # STEERING_LTA does not seem to allow more rate by sending faster, and may wind up easier
    if self.frame % 2 == 0 and self.CP.carFingerprint in TSS2_CAR:
      lta_active = lat_active and self.CP.steerControlType == SteerControlType.angle
      # cut steering torque with TORQUE_WIND_DOWN when either EPS torque or driver torque is above
      # the threshold, to limit max lateral acceleration and for driver torque blending respectively.
      full_torque_condition = (abs(CS.out.steeringTorqueEps) < self.params.STEER_MAX and
                               abs(CS.out.steeringTorque) < MAX_LTA_DRIVER_TORQUE_ALLOWANCE)

      # TORQUE_WIND_DOWN at 0 ramps down torque at roughly the max down rate of 1500 units/sec
      torque_wind_down = 100 if lta_active and full_torque_condition else 0
      can_sends.append(toyotacan.create_lta_steer_command(self.packer, self.CP.steerControlType, self.last_angle,
                                                          lta_active, self.frame // 2, torque_wind_down))

      if self.CP.flags & ToyotaFlags.SECOC.value:
        lta_steer_2 = toyotacan.create_lta_steer_command_2(self.packer, self.frame // 2)
        lta_steer_2 = add_mac(self.secoc_key,
                              int(CS.secoc_synchronization['TRIP_CNT']),
                              int(CS.secoc_synchronization['RESET_CNT']),
                              self.secoc_lta_message_counter,
                              lta_steer_2)
        self.secoc_lta_message_counter += 1
        can_sends.append(lta_steer_2)

    # *** gas and brake ***
    if Params().get_bool("AleSato_CustomCarApi"):
      # PCM compensation Transition Logic (enter only at first positive calculation)
      if CS.out.gasPressed or not CS.out.cruiseState.enabled:
        self.reset_pcm_compensation = True
      if CS.pcm_neutral_force >= 0:
        self.reset_pcm_compensation = False

      # NO_STOP_TIMER_CAR will creep if compensation is applied when stopping or stopped, don't compensate when stopped or stopping
      should_compensate = True
      if self.CP.carFingerprint in NO_STOP_TIMER_CAR and ((CS.out.vEgo <  1e-3 and actuators.accel < 1e-3) or stopping):
        should_compensate = False
      if CC.longActive and should_compensate and not self.reset_pcm_compensation:
        accel_offset = CS.pcm_neutral_force / self.CP.mass
      else:
        accel_offset = 0.
      if not CS.out.gasPressed:
        pcm_accel_cmd = clip(actuators.accel + accel_offset, self.params.ACCEL_MIN, self.params.ACCEL_MAX)
      else:
        pcm_accel_cmd = 0.
    else:
      # For cars where we allow a higher max acceleration of 2.0 m/s^2, compensate for PCM request overshoot and imprecise braking
      # TODO: sometimes when switching from brake to gas quickly, CLUTCH->ACCEL_NET shows a slow unwind. make it go to 0 immediately
      if self.CP.flags & ToyotaFlags.RAISED_ACCEL_LIMIT and CC.longActive and not CS.out.cruiseState.standstill:
        # calculate amount of acceleration PCM should apply to reach target, given pitch
        accel_due_to_pitch = math.sin(CS.slope_angle) * ACCELERATION_DUE_TO_GRAVITY
        net_acceleration_request = actuators.accel + accel_due_to_pitch

        # let PCM handle stopping for now
        pcm_accel_compensation = 0.0
        if actuators.longControlState != LongCtrlState.stopping:
          pcm_accel_compensation = 2.0 * (CS.pcm_accel_net - net_acceleration_request)

        # prevent compensation windup
        pcm_accel_compensation = clip(pcm_accel_compensation, actuators.accel - self.params.ACCEL_MAX,
                                      actuators.accel - self.params.ACCEL_MIN)

        if self.CP.flags & ToyotaFlags.HYBRID:
          self.pcm_accel_compensation = rate_limit(pcm_accel_compensation, self.pcm_accel_compensation, -0.01 * 0.25, 0.01 * 0.25)
        else:
          self.pcm_accel_compensation = rate_limit(pcm_accel_compensation, self.pcm_accel_compensation, -0.01, 0.01)
        pcm_accel_cmd = actuators.accel - self.pcm_accel_compensation

        # Along with rate limiting positive jerk below, this greatly improves gas response time
        # Consider the net acceleration request that the PCM should be applying (pitch included)
        if net_acceleration_request < 0.1:
          self.permit_braking = True
        elif net_acceleration_request > 0.2:
          self.permit_braking = False
      else:
        self.pcm_accel_compensation = 0.0
        pcm_accel_cmd = actuators.accel
        self.permit_braking = True

      pcm_accel_cmd = clip(pcm_accel_cmd, self.params.ACCEL_MIN, self.params.ACCEL_MAX)

    # on entering standstill, send standstill request
    if CS.out.standstill and not self.last_standstill and (self.CP.carFingerprint not in NO_STOP_TIMER_CAR):
      self.standstill_req = True
    if CS.pcm_acc_status != 8:
      # pcm entered standstill or it's disabled
      self.standstill_req = False

    self.last_standstill = CS.out.standstill

    # AleSato Stuff
    def make_can_msg(addr, dat, bus):
      return [addr, dat, bus]
    self.remoteLockDoors = Params().get_bool("AleSato_RemoteLockDoors")
    if self.remoteLockDoors and not self.lastRemoteLockDoors:
      self.oneHonk = True
      self.honk_rate_counter = self.frame
    elif not self.remoteLockDoors and self.lastRemoteLockDoors:
      self.twoHonks = True
      self.honk_rate_counter = self.frame
    self.lastRemoteLockDoors = self.remoteLockDoors
    if self.oneHonk:
      if self.frame == (self.honk_rate_counter + 5):
        can_sends.append(make_can_msg(0x750, HORN_ON_CMD, 0))
      elif self.frame == (self.honk_rate_counter + 6):
        can_sends.append(make_can_msg(0x750, HORN_OFF_CMD, 0))
      elif self.frame > (self.honk_rate_counter + 6):
        can_sends.append(make_can_msg(0x750, LOCK_CMD, 0))
        self.oneHonk = False
    if self.twoHonks:
      if self.frame == (self.honk_rate_counter + 5):
        can_sends.append(make_can_msg(0x750, HORN_ON_CMD, 0))
      elif self.frame == (self.honk_rate_counter + 6):
        can_sends.append(make_can_msg(0x750, HORN_OFF_CMD, 0))
      elif self.frame == (self.honk_rate_counter + 31):
        can_sends.append(make_can_msg(0x750, HORN_ON_CMD, 0))
      elif self.frame == (self.honk_rate_counter + 32):
        can_sends.append(make_can_msg(0x750, HORN_OFF_CMD, 0))
      elif self.frame > (self.honk_rate_counter + 32):
        can_sends.append(make_can_msg(0x750, UNLOCK_CMD, 0))
        self.twoHonks = False

    # AleSato's Automatic Brake Hold
    if self.frame % 2 == 0:
      if CS.out.brakeholdGovernor:
        can_sends.append(toyotacan.create_brakehold_command(self.packer, {}, True if self.frame % 730 < 727 else False))
      else:
        can_sends.append(toyotacan.create_brakehold_command(self.packer, CS.stock_aeb, False))

    # handle UI messages
    fcw_alert = hud_control.visualAlert == VisualAlert.fcw
    steer_alert = hud_control.visualAlert in (VisualAlert.steerRequired, VisualAlert.ldw)

    # we can spam can to cancel the system even if we are using lat only control
    if (self.frame % 3 == 0 and self.CP.openpilotLongitudinalControl) or pcm_cancel_cmd:
      lead = hud_control.leadVisible or CS.out.vEgo < 12.  # at low speed we always assume the lead is present so ACC can be engaged

      # Press distance button until we are at the correct bar length. Only change while enabled to avoid skipping startup popup
      if self.frame % 6 == 0 and self.CP.openpilotLongitudinalControl:
        desired_distance = 4 - hud_control.leadDistanceBars
        if CS.out.cruiseState.enabled and CS.pcm_follow_distance != desired_distance:
          self.distance_button = not self.distance_button
        else:
          self.distance_button = 0

      # Lexus IS uses a different cancellation message
      if pcm_cancel_cmd and self.CP.carFingerprint in UNSUPPORTED_DSU_CAR:
        can_sends.append(toyotacan.create_acc_cancel_command(self.packer))
      elif self.CP.openpilotLongitudinalControl:
        if Params().get_bool("AleSato_CustomCarApi"):
          # AleSato apply in a diff way the neutralForce compensation than Irene's (Cydia2020)
          accel_raw = -0.4 if stopping else actuators.accel if should_compensate else pcm_accel_cmd
          can_sends.append(toyotacan.create_my_accel_command(self.packer, pcm_accel_cmd, accel_raw, stopping, pcm_cancel_cmd, self.standstill_req, \
                                                          lead, CS.acc_type, self.distance_button, fcw_alert))
        else:
          # internal PCM gas command can get stuck unwinding from negative accel so we apply a generous rate limit
          pcm_accel_cmd = min(pcm_accel_cmd, self.accel + ACCEL_WINDUP_LIMIT) if CC.longActive else 0.0

          can_sends.append(toyotacan.create_accel_command(self.packer, pcm_accel_cmd, pcm_cancel_cmd, self.permit_braking, self.standstill_req, lead,
                                                          CS.acc_type, fcw_alert, self.distance_button))
        self.accel = pcm_accel_cmd
      else:
        if Params().get_bool("AleSato_CustomCarApi"):
          can_sends.append(toyotacan.create_my_accel_command(self.packer, 0, 0, True, pcm_cancel_cmd, False, lead, CS.acc_type, self.distance_button, False))
        else:
          can_sends.append(toyotacan.create_accel_command(self.packer, 0, pcm_cancel_cmd, True, False, lead, CS.acc_type, False, self.distance_button))

    # *** hud ui ***
    if self.CP.carFingerprint != CAR.TOYOTA_PRIUS_V:
      # ui mesg is at 1Hz but we send asap if:
      # - there is something to display
      # - there is something to stop displaying
      send_ui = False
      if ((fcw_alert or steer_alert) and not self.alert_active) or \
         (not (fcw_alert or steer_alert) and self.alert_active):
        send_ui = True
        self.alert_active = not self.alert_active
      elif pcm_cancel_cmd:
        # forcing the pcm to disengage causes a bad fault sound so play a good sound instead
        send_ui = True

      if self.frame % 20 == 0 or send_ui:
        # can_sends.append(toyotacan.create_ui_command(self.packer, steer_alert, pcm_cancel_cmd, hud_control.leftLaneVisible,
        #                                              hud_control.rightLaneVisible, hud_control.leftLaneDepart,
        #                                              hud_control.rightLaneDepart, CC.enabled, CS.lkas_hud))
        # AleSato handle InstrumentCluster in a different way to display MADS status properly:
        can_sends.append(toyotacan.create_ui_command(self.packer, steer_alert, pcm_cancel_cmd, hud_control.leftLaneVisible,
                                                      hud_control.rightLaneVisible, hud_control.leftLaneDepart,
                                                      hud_control.rightLaneDepart, CC.enabled, CS.lkas_hud, CS.out.madsEnabled))

      if (self.frame % 100 == 0 or send_ui) and (self.CP.enableDsu or self.CP.flags & ToyotaFlags.DISABLE_RADAR.value):
        can_sends.append(toyotacan.create_fcw_command(self.packer, fcw_alert))

    # *** static msgs ***
    for addr, cars, bus, fr_step, vl in STATIC_DSU_MSGS:
      if self.frame % fr_step == 0 and self.CP.enableDsu and self.CP.carFingerprint in cars:
        can_sends.append(CanData(addr, vl, bus))

    # keep radar disabled
    if self.frame % 20 == 0 and self.CP.flags & ToyotaFlags.DISABLE_RADAR.value:
      can_sends.append(make_tester_present_msg(0x750, 0, 0xF))

    new_actuators = actuators.as_builder()
    new_actuators.steer = apply_steer / self.params.STEER_MAX
    new_actuators.steerOutputCan = apply_steer
    new_actuators.steeringAngleDeg = self.last_angle
    new_actuators.accel = self.accel

    self.frame += 1
    return new_actuators, can_sends
