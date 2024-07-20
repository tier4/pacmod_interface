// Copyright 2017-2019 Autoware Foundation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <pacmod_interface/pacmod_interface.hpp>
#include <vehicle_info_util/vehicle_info_util.hpp>

#include <algorithm>
#include <limits>
#include <memory>
#include <utility>

PacmodInterface::PacmodInterface() : Node("pacmod_interface")
{
  /* setup parameters */
  base_frame_id_ = declare_parameter("base_frame_id", "base_link");
  command_timeout_ms_ = declare_parameter("command_timeout_ms", 1000);
  loop_rate_ = declare_parameter("loop_rate", 30.0);

  /* parameters for vehicle specifications */
  vehicle_info_ = vehicle_info_util::VehicleInfoUtil(*this).getVehicleInfo();
  tire_radius_ = vehicle_info_.wheel_radius_m;
  wheel_base_ = vehicle_info_.wheel_base_m;

  steering_offset_ = declare_parameter("steering_offset", 0.0);
  enable_steering_rate_control_ = declare_parameter("enable_steering_rate_control", false);

  /* vehicle parameters */
  vgr_coef_a_ = declare_parameter("vgr_coef_a", 15.713);
  vgr_coef_b_ = declare_parameter("vgr_coef_b", 0.053);
  vgr_coef_c_ = declare_parameter("vgr_coef_c", 0.042);
  accel_pedal_offset_ = declare_parameter("accel_pedal_offset", 0.0);
  brake_pedal_offset_ = declare_parameter("brake_pedal_offset", 0.0);

  /* parameters for limitter */
  max_throttle_ = declare_parameter("max_throttle", 0.2);
  max_brake_ = declare_parameter("max_brake", 0.8);
  max_steering_wheel_ = declare_parameter("max_steering_wheel", 2.7 * M_PI);
  max_steering_wheel_rate_ = declare_parameter("max_steering_wheel_rate", 6.6);
  min_steering_wheel_rate_ = declare_parameter("min_steering_wheel_rate", 0.5);
  steering_wheel_rate_low_vel_ = declare_parameter("steering_wheel_rate_low_vel", 5.0);
  steering_wheel_rate_stopped_ = declare_parameter("steering_wheel_rate_stopped", 5.0);
  low_vel_thresh_ = declare_parameter("low_vel_thresh", 1.389);  // 5.0kmh

  /* parameters for turn signal recovery */
  hazard_thresh_time_ = declare_parameter("hazard_thresh_time", 0.20);  // s

  /* parameter for preventing gear chattering */
  margin_time_for_gear_change_ = declare_parameter("margin_time_for_gear_change", 2.0);

  /* initialize */
  prev_steer_cmd_.header.stamp = this->now();
  prev_steer_cmd_.command = 0.0;

  /* subscribers */
  using std::placeholders::_1;
  using std::placeholders::_2;

  // From pacmod
  rear_door_rpt_sub_ = create_subscription<pacmod3_msgs::msg::SystemRptInt>(
    "/pacmod/rear_pass_door_rpt", 1, std::bind(&PacmodInterface::callbackRearDoor, this, _1));

  steer_wheel_rpt_sub_ =
    std::make_unique<message_filters::Subscriber<pacmod3_msgs::msg::SystemRptFloat>>(
      this, "/pacmod/steering_rpt");
  wheel_speed_rpt_sub_ =
    std::make_unique<message_filters::Subscriber<pacmod3_msgs::msg::WheelSpeedRpt>>(
      this, "/pacmod/wheel_speed_rpt");
  accel_rpt_sub_ = std::make_unique<message_filters::Subscriber<pacmod3_msgs::msg::SystemRptFloat>>(
    this, "/pacmod/accel_rpt");
  brake_rpt_sub_ = std::make_unique<message_filters::Subscriber<pacmod3_msgs::msg::SystemRptFloat>>(
    this, "/pacmod/brake_rpt");
  shift_rpt_sub_ = std::make_unique<message_filters::Subscriber<pacmod3_msgs::msg::SystemRptInt>>(
    this, "/pacmod/shift_rpt");
  turn_rpt_sub_ = std::make_unique<message_filters::Subscriber<pacmod3_msgs::msg::SystemRptInt>>(
    this, "/pacmod/turn_rpt");
  global_rpt_sub_ = std::make_unique<message_filters::Subscriber<pacmod3_msgs::msg::GlobalRpt>>(
    this, "/pacmod/global_rpt");

  pacmod_feedbacks_sync_ =
    std::make_unique<message_filters::Synchronizer<PacmodFeedbacksSyncPolicy>>(
      PacmodFeedbacksSyncPolicy(10), *steer_wheel_rpt_sub_, *wheel_speed_rpt_sub_, *accel_rpt_sub_,
      *brake_rpt_sub_, *shift_rpt_sub_, *turn_rpt_sub_, *global_rpt_sub_);

  pacmod_feedbacks_sync_->registerCallback(std::bind(
    &PacmodInterface::callbackPacmodRpt, this, std::placeholders::_1, std::placeholders::_2,
    std::placeholders::_3, std::placeholders::_4, std::placeholders::_5, std::placeholders::_6,
    std::placeholders::_7));

  /* publisher */
  // To pacmod
  accel_cmd_pub_ =
    create_publisher<pacmod3_msgs::msg::SystemCmdFloat>("/pacmod/accel_cmd", rclcpp::QoS{1});
  brake_cmd_pub_ =
    create_publisher<pacmod3_msgs::msg::SystemCmdFloat>("/pacmod/brake_cmd", rclcpp::QoS{1});
  steer_cmd_pub_ =
    create_publisher<pacmod3_msgs::msg::SteeringCmd>("/pacmod/steering_cmd", rclcpp::QoS{1});
  shift_cmd_pub_ =
    create_publisher<pacmod3_msgs::msg::SystemCmdInt>("/pacmod/shift_cmd", rclcpp::QoS{1});
  turn_cmd_pub_ =
    create_publisher<pacmod3_msgs::msg::SystemCmdInt>("/pacmod/turn_cmd", rclcpp::QoS{1});
  door_cmd_pub_ =
    create_publisher<pacmod3_msgs::msg::SystemCmdInt>("/pacmod/rear_pass_door_cmd", rclcpp::QoS{1});
  raw_steer_cmd_pub_ = create_publisher<pacmod3_msgs::msg::SteeringCmd>(
    "/pacmod/raw_steer_cmd", rclcpp::QoS{1});  // only for debug

  // To Autoware
  control_mode_pub_ = create_publisher<autoware_vehicle_msgs::msg::ControlModeReport>(
    "/vehicle/status/control_mode", rclcpp::QoS{1});
  vehicle_twist_pub_ = create_publisher<autoware_vehicle_msgs::msg::VelocityReport>(
    "/vehicle/status/velocity_status", rclcpp::QoS{1});
  steering_status_pub_ = create_publisher<autoware_vehicle_msgs::msg::SteeringReport>(
    "/vehicle/status/steering_status", rclcpp::QoS{1});
  gear_status_pub_ = create_publisher<autoware_vehicle_msgs::msg::GearReport>(
    "/vehicle/status/gear_status", rclcpp::QoS{1});
  turn_indicators_status_pub_ = create_publisher<autoware_vehicle_msgs::msg::TurnIndicatorsReport>(
    "/vehicle/status/turn_indicators_status", rclcpp::QoS{1});
  hazard_lights_status_pub_ = create_publisher<autoware_vehicle_msgs::msg::HazardLightsReport>(
    "/vehicle/status/hazard_lights_status", rclcpp::QoS{1});
  actuation_status_pub_ =
    create_publisher<ActuationStatusStamped>("/vehicle/status/actuation_status", 1);
  steering_wheel_status_pub_ =
    create_publisher<SteeringWheelStatusStamped>("/vehicle/status/steering_wheel_status", 1);
  door_status_pub_ =
    create_publisher<tier4_api_msgs::msg::DoorStatus>("/vehicle/status/door_status", 1);

}

void PacmodInterface::callbackRearDoor(
  const pacmod3_msgs::msg::SystemRptInt::ConstSharedPtr rear_door_rpt)
{
  /* publish current door status */
  door_status_pub_->publish(toAutowareDoorStatusMsg(*rear_door_rpt));
}

void PacmodInterface::callbackPacmodRpt(
  const pacmod3_msgs::msg::SystemRptFloat::ConstSharedPtr steer_wheel_rpt,
  const pacmod3_msgs::msg::WheelSpeedRpt::ConstSharedPtr wheel_speed_rpt,
  const pacmod3_msgs::msg::SystemRptFloat::ConstSharedPtr accel_rpt,
  const pacmod3_msgs::msg::SystemRptFloat::ConstSharedPtr brake_rpt,
  const pacmod3_msgs::msg::SystemRptInt::ConstSharedPtr shift_rpt,
  const pacmod3_msgs::msg::SystemRptInt::ConstSharedPtr turn_rpt,
  const pacmod3_msgs::msg::GlobalRpt::ConstSharedPtr global_rpt)
{
  is_pacmod_rpt_received_ = true;
  steer_wheel_rpt_ptr_ = steer_wheel_rpt;
  wheel_speed_rpt_ptr_ = wheel_speed_rpt;
  accel_rpt_ptr_ = accel_rpt;
  brake_rpt_ptr_ = brake_rpt;
  gear_cmd_rpt_ptr_ = shift_rpt;
  global_rpt_ptr_ = global_rpt;
  turn_rpt_ptr_ = turn_rpt;

  is_pacmod_enabled_ =
    steer_wheel_rpt_ptr_->enabled && accel_rpt_ptr_->enabled && brake_rpt_ptr_->enabled;
  RCLCPP_DEBUG(
    get_logger(),
    "enabled: is_pacmod_enabled_ %d, steer %d, accel %d, brake %d, shift %d, "
    "global %d",
    is_pacmod_enabled_, steer_wheel_rpt_ptr_->enabled, accel_rpt_ptr_->enabled,
    brake_rpt_ptr_->enabled, gear_cmd_rpt_ptr_->enabled, global_rpt_ptr_->enabled);

  const double current_velocity = calculateVehicleVelocity(
    *wheel_speed_rpt_ptr_, *gear_cmd_rpt_ptr_);  // current vehicle speed > 0 [m/s]
  const double current_steer_wheel =
    steer_wheel_rpt_ptr_->output;  // current vehicle steering wheel angle [rad]
  const double adaptive_gear_ratio =
    calculateVariableGearRatio(current_velocity, current_steer_wheel);
  const double current_steer = current_steer_wheel / adaptive_gear_ratio + steering_offset_;

  std_msgs::msg::Header header;
  header.frame_id = base_frame_id_;
  header.stamp = get_clock()->now();

  /* publish steering wheel status */
  {
    SteeringWheelStatusStamped steering_wheel_status_msg;
    steering_wheel_status_msg.stamp = header.stamp;
    steering_wheel_status_msg.data = current_steer_wheel;
    steering_wheel_status_pub_->publish(steering_wheel_status_msg);
  }

  /* publish vehicle status control_mode */
  {
    autoware_vehicle_msgs::msg::ControlModeReport control_mode_msg;
    control_mode_msg.stamp = header.stamp;

    if (global_rpt->enabled && is_pacmod_enabled_) {
      control_mode_msg.mode = autoware_vehicle_msgs::msg::ControlModeReport::AUTONOMOUS;
    } else {
      control_mode_msg.mode = autoware_vehicle_msgs::msg::ControlModeReport::MANUAL;
    }

    control_mode_pub_->publish(control_mode_msg);
  }

  /* publish vehicle status twist */
  {
    autoware_vehicle_msgs::msg::VelocityReport twist;
    twist.header = header;
    twist.longitudinal_velocity = current_velocity;                                 // [m/s]
    twist.heading_rate = current_velocity * std::tan(current_steer) / wheel_base_;  // [rad/s]
    vehicle_twist_pub_->publish(twist);
  }

  /* publish current shift */
  {
    autoware_vehicle_msgs::msg::GearReport gear_report_msg;
    gear_report_msg.stamp = header.stamp;
    const auto opt_gear_report = toAutowareShiftReport(*gear_cmd_rpt_ptr_);
    if (opt_gear_report) {
      gear_report_msg.report = *opt_gear_report;
      gear_status_pub_->publish(gear_report_msg);
    }
  }

  /* publish current status */
  {
    autoware_vehicle_msgs::msg::SteeringReport steer_msg;
    steer_msg.stamp = header.stamp;
    steer_msg.steering_tire_angle = current_steer;
    steering_status_pub_->publish(steer_msg);
  }

  /* publish control status */
  {
    ActuationStatusStamped actuation_status;
    actuation_status.header = header;
    actuation_status.status.accel_status = accel_rpt_ptr_->output;
    actuation_status.status.brake_status = brake_rpt_ptr_->output;
    actuation_status.status.steer_status = current_steer;
    actuation_status_pub_->publish(actuation_status);
  }

  /* publish current turn signal */
  {
    autoware_vehicle_msgs::msg::TurnIndicatorsReport turn_msg;
    turn_msg.stamp = header.stamp;
    turn_msg.report = toAutowareTurnIndicatorsReport(*turn_rpt);
    turn_indicators_status_pub_->publish(turn_msg);

    autoware_vehicle_msgs::msg::HazardLightsReport hazard_msg;
    hazard_msg.stamp = header.stamp;
    hazard_msg.report = toAutowareHazardLightsReport(*turn_rpt);
    hazard_lights_status_pub_->publish(hazard_msg);
  }
}

double PacmodInterface::calculateVehicleVelocity(
  const pacmod3_msgs::msg::WheelSpeedRpt & wheel_speed_rpt,
  const pacmod3_msgs::msg::SystemRptInt & shift_rpt)
{
  const double sign = (shift_rpt.output == pacmod3_msgs::msg::SystemRptInt::SHIFT_REVERSE) ? -1 : 1;
  const double vel =
    (wheel_speed_rpt.rear_left_wheel_speed + wheel_speed_rpt.rear_right_wheel_speed) * 0.5 *
    tire_radius_;
  return sign * vel;
}

double PacmodInterface::calculateVariableGearRatio(const double vel, const double steer_wheel)
{
  return std::max(
    1e-5, vgr_coef_a_ + vgr_coef_b_ * vel * vel - vgr_coef_c_ * std::fabs(steer_wheel));
}

uint16_t PacmodInterface::getGearCmdForPreventChatter(uint16_t gear_command)
{
  // first time to change gear
  if (!last_time_to_change_gear_ptr_) {
    // send gear change command
    last_time_to_change_gear_ptr_ = std::make_shared<rclcpp::Time>(this->now());
    prev_gear_command_ = gear_command;
    return gear_command;
  }

  // no gear change
  if (gear_command == prev_gear_command_) {
    return gear_command;
  }

  const auto time_from_last_gear_change = (this->now() - *last_time_to_change_gear_ptr_).seconds();
  if (time_from_last_gear_change < margin_time_for_gear_change_) {
    // hold current gear
    RCLCPP_INFO_STREAM(get_logger(), "current_gear_command: " << static_cast<int>(gear_command));
    RCLCPP_INFO_STREAM(get_logger(), "prev_gear_command: " << static_cast<int>(prev_gear_command_));
    RCLCPP_INFO_STREAM(get_logger(), "send prev_gear_command for preventing gear-chattering");

    return prev_gear_command_;
  }
  // send gear change command
  last_time_to_change_gear_ptr_ = std::make_shared<rclcpp::Time>(this->now());
  prev_gear_command_ = gear_command;
  return gear_command;
}

std::optional<int32_t> PacmodInterface::toAutowareShiftReport(
  const pacmod3_msgs::msg::SystemRptInt & shift)
{
  using autoware_vehicle_msgs::msg::GearReport;
  using pacmod3_msgs::msg::SystemRptInt;

  if (shift.output == SystemRptInt::SHIFT_PARK) {
    return GearReport::PARK;
  }
  if (shift.output == SystemRptInt::SHIFT_REVERSE) {
    return GearReport::REVERSE;
  }
  if (shift.output == SystemRptInt::SHIFT_FORWARD) {
    return GearReport::DRIVE;
  }
  if (shift.output == SystemRptInt::SHIFT_LOW) {
    return GearReport::LOW;
  }
  return {};
}

int32_t PacmodInterface::toAutowareTurnIndicatorsReport(
  const pacmod3_msgs::msg::SystemRptInt & turn)
{
  using autoware_vehicle_msgs::msg::TurnIndicatorsReport;
  using pacmod3_msgs::msg::SystemRptInt;

  if (turn.output == SystemRptInt::TURN_RIGHT) {
    return TurnIndicatorsReport::ENABLE_RIGHT;
  } else if (turn.output == SystemRptInt::TURN_LEFT) {
    return TurnIndicatorsReport::ENABLE_LEFT;
  } else if (turn.output == SystemRptInt::TURN_NONE) {
    return TurnIndicatorsReport::DISABLE;
  }
  return TurnIndicatorsReport::DISABLE;
}

int32_t PacmodInterface::toAutowareHazardLightsReport(
  const pacmod3_msgs::msg::SystemRptInt & hazard)
{
  using autoware_vehicle_msgs::msg::HazardLightsReport;
  using pacmod3_msgs::msg::SystemRptInt;

  if (hazard.output == SystemRptInt::TURN_HAZARDS) {
    return HazardLightsReport::ENABLE;
  }

  return HazardLightsReport::DISABLE;
}


tier4_api_msgs::msg::DoorStatus PacmodInterface::toAutowareDoorStatusMsg(
  const pacmod3_msgs::msg::SystemRptInt & msg_ptr)
{
  using pacmod3_msgs::msg::SystemRptInt;
  using tier4_api_msgs::msg::DoorStatus;
  DoorStatus door_status;

  door_status.status = DoorStatus::UNKNOWN;

  if (msg_ptr.command == SystemRptInt::DOOR_CLOSE && msg_ptr.output == SystemRptInt::DOOR_OPEN) {
    // do not used (command & output are always the same value)
    door_status.status = DoorStatus::DOOR_CLOSING;
  } else if (  // NOLINT
    msg_ptr.command == SystemRptInt::DOOR_OPEN && msg_ptr.output == SystemRptInt::DOOR_CLOSE) {
    // do not used (command & output are always the same value)
    door_status.status = DoorStatus::DOOR_OPENING;
  } else if (msg_ptr.output == SystemRptInt::DOOR_CLOSE) {
    door_status.status = DoorStatus::DOOR_CLOSED;
  } else if (msg_ptr.output == SystemRptInt::DOOR_OPEN) {
    door_status.status = DoorStatus::DOOR_OPENED;
  }

  return door_status;
}
