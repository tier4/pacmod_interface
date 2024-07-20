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

#ifndef PACMOD_INTERFACE__PACMOD_INTERFACE_HPP_
#define PACMOD_INTERFACE__PACMOD_INTERFACE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <vehicle_info_util/vehicle_info_util.hpp>

#include <pacmod3_msgs/msg/global_rpt.hpp>
#include <pacmod3_msgs/msg/steering_cmd.hpp>
#include <pacmod3_msgs/msg/system_cmd_float.hpp>
#include <pacmod3_msgs/msg/system_cmd_int.hpp>
#include <pacmod3_msgs/msg/system_rpt_float.hpp>
#include <pacmod3_msgs/msg/system_rpt_int.hpp>
#include <pacmod3_msgs/msg/wheel_speed_rpt.hpp>
#include <vehicle_msgs/msg/actuation_status_stamped.hpp>
#include <vehicle_msgs/msg/control_mode_report.hpp>
#include <vehicle_msgs/msg/door_status.hpp>
#include <vehicle_msgs/msg/gear_report.hpp>
#include <vehicle_msgs/msg/hazard_lights_report.hpp>
#include <vehicle_msgs/msg/steering_report.hpp>
#include <vehicle_msgs/msg/steering_wheel_status_stamped.hpp>
#include <vehicle_msgs/msg/turn_indicators_report.hpp>
#include <vehicle_msgs/msg/vehicle_emergency_stamped.hpp>
#include <vehicle_msgs/msg/velocity_report.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <memory>
#include <optional>
#include <string>

class PacmodInterface : public rclcpp::Node
{
public:
  PacmodInterface();

private:
  typedef message_filters::sync_policies::ApproximateTime<
    pacmod3_msgs::msg::SystemRptFloat, pacmod3_msgs::msg::WheelSpeedRpt,
    pacmod3_msgs::msg::SystemRptFloat, pacmod3_msgs::msg::SystemRptFloat,
    pacmod3_msgs::msg::SystemRptInt, pacmod3_msgs::msg::SystemRptInt, pacmod3_msgs::msg::GlobalRpt>
    PacmodFeedbacksSyncPolicy;

  /* subscribers */

  // From Pacmod
  rclcpp::Subscription<pacmod3_msgs::msg::SystemRptInt>::SharedPtr rear_door_rpt_sub_;

  std::unique_ptr<message_filters::Subscriber<pacmod3_msgs::msg::SystemRptFloat>>
    steer_wheel_rpt_sub_;
  std::unique_ptr<message_filters::Subscriber<pacmod3_msgs::msg::WheelSpeedRpt>>
    wheel_speed_rpt_sub_;
  std::unique_ptr<message_filters::Subscriber<pacmod3_msgs::msg::SystemRptFloat>> accel_rpt_sub_;
  std::unique_ptr<message_filters::Subscriber<pacmod3_msgs::msg::SystemRptFloat>> brake_rpt_sub_;
  std::unique_ptr<message_filters::Subscriber<pacmod3_msgs::msg::SystemRptInt>> shift_rpt_sub_;
  std::unique_ptr<message_filters::Subscriber<pacmod3_msgs::msg::SystemRptInt>> turn_rpt_sub_;
  std::unique_ptr<message_filters::Subscriber<pacmod3_msgs::msg::GlobalRpt>> global_rpt_sub_;
  std::unique_ptr<message_filters::Synchronizer<PacmodFeedbacksSyncPolicy>> pacmod_feedbacks_sync_;

  /* publishers */
  // To Pacmod
  rclcpp::Publisher<pacmod3_msgs::msg::SystemCmdFloat>::SharedPtr accel_cmd_pub_;
  rclcpp::Publisher<pacmod3_msgs::msg::SystemCmdFloat>::SharedPtr brake_cmd_pub_;
  rclcpp::Publisher<pacmod3_msgs::msg::SteeringCmd>::SharedPtr steer_cmd_pub_;
  rclcpp::Publisher<pacmod3_msgs::msg::SystemCmdInt>::SharedPtr shift_cmd_pub_;
  rclcpp::Publisher<pacmod3_msgs::msg::SystemCmdInt>::SharedPtr turn_cmd_pub_;
  rclcpp::Publisher<pacmod3_msgs::msg::SystemCmdInt>::SharedPtr door_cmd_pub_;
  rclcpp::Publisher<pacmod3_msgs::msg::SteeringCmd>::SharedPtr
    raw_steer_cmd_pub_;  // only for debug

  // To Autoware
  rclcpp::Publisher<vehicle_msgs::msg::ControlModeReport>::SharedPtr control_mode_pub_;
  rclcpp::Publisher<vehicle_msgs::msg::VelocityReport>::SharedPtr vehicle_twist_pub_;
  rclcpp::Publisher<vehicle_msgs::msg::SteeringReport>::SharedPtr steering_status_pub_;
  rclcpp::Publisher<vehicle_msgs::msg::GearReport>::SharedPtr gear_status_pub_;
  rclcpp::Publisher<vehicle_msgs::msg::TurnIndicatorsReport>::SharedPtr turn_indicators_status_pub_;
  rclcpp::Publisher<vehicle_msgs::msg::HazardLightsReport>::SharedPtr hazard_lights_status_pub_;
  rclcpp::Publisher<vehicle_msgs::msg::ActuationStatusStamped>::SharedPtr actuation_status_pub_;
  rclcpp::Publisher<vehicle_msgs::msg::SteeringWheelStatusStamped>::SharedPtr
    steering_wheel_status_pub_;
  rclcpp::Publisher<vehicle_msgs::msg::DoorStatus>::SharedPtr door_status_pub_;

  /* ros param */
  std::string base_frame_id_;
  int command_timeout_ms_;  // vehicle_cmd timeout [ms]
  bool is_pacmod_rpt_received_ = false;
  bool is_pacmod_enabled_ = false;
  bool is_clear_override_needed_ = false;
  bool prev_override_ = false;
  double loop_rate_;           // [Hz]
  double tire_radius_;         // [m]
  double wheel_base_;          // [m]
  double steering_offset_;     // [rad] def: measured = truth + offset
  double vgr_coef_a_;          // variable gear ratio coeffs
  double vgr_coef_b_;          // variable gear ratio coeffs
  double vgr_coef_c_;          // variable gear ratio coeffs
  double accel_pedal_offset_;  // offset of accel pedal value
  double brake_pedal_offset_;  // offset of brake pedal value

  double max_throttle_;                 // max throttle [0~1]
  double max_brake_;                    // max throttle [0~1]
  double max_steering_wheel_;           // max steering wheel angle [rad]
  double max_steering_wheel_rate_;      // [rad/s]
  double min_steering_wheel_rate_;      // [rad/s]
  double steering_wheel_rate_low_vel_;  // [rad/s]
  double steering_wheel_rate_stopped_;  // [rad/s]
  double low_vel_thresh_;               // [m/s]

  bool enable_steering_rate_control_;  // use steering angle speed for command [rad/s]

  double hazard_thresh_time_;
  int hazard_recover_count_ = 0;
  const int hazard_recover_cmd_num_ = 5;

  double margin_time_for_gear_change_;  // [s]

  vehicle_info_util::VehicleInfo vehicle_info_;

  /* input values */
  pacmod3_msgs::msg::SystemRptFloat::ConstSharedPtr steer_wheel_rpt_ptr_;  // [rad]
  pacmod3_msgs::msg::WheelSpeedRpt::ConstSharedPtr wheel_speed_rpt_ptr_;   // [m/s]
  pacmod3_msgs::msg::SystemRptFloat::ConstSharedPtr accel_rpt_ptr_;
  pacmod3_msgs::msg::SystemRptFloat::ConstSharedPtr brake_rpt_ptr_;   // [m/s]
  pacmod3_msgs::msg::SystemRptInt::ConstSharedPtr gear_cmd_rpt_ptr_;  // [m/s]
  pacmod3_msgs::msg::GlobalRpt::ConstSharedPtr global_rpt_ptr_;       // [m/s]
  pacmod3_msgs::msg::SystemRptInt::ConstSharedPtr turn_rpt_ptr_;
  pacmod3_msgs::msg::SteeringCmd prev_steer_cmd_;

  std::shared_ptr<rclcpp::Time> last_time_to_change_gear_ptr_;
  uint16_t prev_gear_command_ = pacmod3_msgs::msg::SystemCmdInt::SHIFT_PARK;

  /* callbacks */
  void callbackRearDoor(const pacmod3_msgs::msg::SystemRptInt::ConstSharedPtr rear_door_rpt);
  void callbackPacmodRpt(
    const pacmod3_msgs::msg::SystemRptFloat::ConstSharedPtr steer_wheel_rpt,
    const pacmod3_msgs::msg::WheelSpeedRpt::ConstSharedPtr wheel_speed_rpt,
    const pacmod3_msgs::msg::SystemRptFloat::ConstSharedPtr accel_rpt,
    const pacmod3_msgs::msg::SystemRptFloat::ConstSharedPtr brake_rpt,
    const pacmod3_msgs::msg::SystemRptInt::ConstSharedPtr gear_cmd_rpt,
    const pacmod3_msgs::msg::SystemRptInt::ConstSharedPtr turn_rpt,
    const pacmod3_msgs::msg::GlobalRpt::ConstSharedPtr global_rpt);

  /*  functions */
  double calculateVehicleVelocity(
    const pacmod3_msgs::msg::WheelSpeedRpt & wheel_speed_rpt,
    const pacmod3_msgs::msg::SystemRptInt & shift_rpt);
  double calculateVariableGearRatio(const double vel, const double steer_wheel);
  uint16_t getGearCmdForPreventChatter(uint16_t gear_command);

  std::optional<int32_t> toAutowareShiftReport(const pacmod3_msgs::msg::SystemRptInt & shift);
  int32_t toAutowareTurnIndicatorsReport(const pacmod3_msgs::msg::SystemRptInt & turn);
  int32_t toAutowareHazardLightsReport(const pacmod3_msgs::msg::SystemRptInt & turn);
  pacmod3_msgs::msg::SystemCmdInt createClearOverrideDoorCommand();
  pacmod3_msgs::msg::SystemCmdInt createDoorCommand(const bool open);
  vehicle_msgs::msg::DoorStatus toAutowareDoorStatusMsg(
    const pacmod3_msgs::msg::SystemRptInt & msg_ptr);
};

#endif  // PACMOD_INTERFACE__PACMOD_INTERFACE_HPP_
