#include "input_mapper.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

namespace rosplane
{

InputMapper::InputMapper()
    : Node("input_mapper")
    , roll_override_(false)
    , pitch_override_(false)
    , param_change_pending_(false)
    , params_(this)
{
  mapped_controller_commands_pub_ =
    this->create_publisher<rosplane_msgs::msg::ControllerCommands>("mapped_controller_command", 10);
  mapped_command_pub_ = this->create_publisher<rosflight_msgs::msg::Command>("mapped_command", 10);

  controller_commands_sub_ = this->create_subscription<rosplane_msgs::msg::ControllerCommands>(
    "input_controller_command", 10,
    std::bind(&InputMapper::controller_commands_callback, this, _1));
  command_sub_ = this->create_subscription<rosflight_msgs::msg::Command>(
    "input_command", 10, std::bind(&InputMapper::command_callback, this, _1));
  rc_raw_sub_ = this->create_subscription<rosflight_msgs::msg::RCRaw>(
    "rc_raw", 10, std::bind(&InputMapper::rc_raw_callback, this, _1));
  state_sub_ = this->create_subscription<rosplane_msgs::msg::State>(
    "estimated_state", 10, std::bind(&InputMapper::state_callback, this, _1));

  set_param_client_ = this->create_client<rcl_interfaces::srv::SetParameters>(
    "/autopilot/set_parameters", rmw_qos_profile_services_default);
  set_param_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(100), std::bind(&InputMapper::set_param_timer_callback, this));
  set_param_timer_->cancel();

  path_follower_mode_service_ = this->create_service<std_srvs::srv::Trigger>(
    "/input_mapper/set_path_follower_mode",
    std::bind(&InputMapper::path_follower_mode_callback, this, _1, _2));
  altitude_course_airspeed_control_mode_service_ = this->create_service<std_srvs::srv::Trigger>(
    "/input_mapper/set_altitude_course_airspeed_control_mode",
    std::bind(&InputMapper::altitude_course_airspeed_control_mode_callback, this, _1, _2));
  angle_control_mode_service_ = this->create_service<std_srvs::srv::Trigger>(
    "/input_mapper/set_angle_control_mode",
    std::bind(&InputMapper::angle_control_mode_callback, this, _1, _2));
  rc_passthrough_mode_service_ = this->create_service<std_srvs::srv::Trigger>(
    "/input_mapper/set_rc_passthrough_mode",
    std::bind(&InputMapper::rc_passthrough_mode_callback, this, _1, _2));

  last_command_time_ = this->now();
  mapped_controller_commands_msg_ = std::make_shared<rosplane_msgs::msg::ControllerCommands>();
  rc_raw_msg_ = std::make_shared<rosflight_msgs::msg::RCRaw>();
  state_msg_ = std::make_shared<rosplane_msgs::msg::State>();

  /// Parameters stuff

  params_.declare_string("aileron_input", "path_follower");
  params_.declare_string("elevator_input", "path_follower");
  params_.declare_string("throttle_input", "path_follower");
  params_.declare_string("rudder_input", "yaw_damper");
  params_.declare_double("rc_roll_angle_min_max", 0.5);
  params_.declare_double("rc_course_rate", 0.5);
  params_.declare_double("rc_pitch_angle_min_max", 0.5);
  params_.declare_double("rc_altitude_rate", 3.0);
  params_.declare_double("rc_airspeed_rate", 1.0);
  params_.declare_double("deadzone_size", 0.05);
  params_.declare_double("max_course_diff_command", 0.25);
  params_.declare_double("max_altitude_diff_command", 10);
  params_.declare_double("max_airspeed_diff_command", 5);

  // Set the parameter callback, for when parameters are changed.
  parameter_callback_handle_ =
    this->add_on_set_parameters_callback(std::bind(&InputMapper::parametersCallback, this, _1));
}

void InputMapper::set_param_timer_callback()
{
  // Check that the service is ready
  if (!set_param_client_->service_is_ready()) {
    RCLCPP_INFO(this->get_logger(), "/autopilot/set_parameters service not available, waiting...");
    return;
  }

  // Send the request
  if (set_param_request_ != nullptr) {
    set_param_future_ = set_param_client_->async_send_request(set_param_request_).share();
    set_param_request_ = nullptr;
    return;
  }

  // Wait for the response
  if (set_param_future_.wait_for(std::chrono::seconds(0)) != std::future_status::ready) {
    return;
  }

  // Check that the parameter was set successfully
  for (auto & result : set_param_future_.get()->results) {
    if (!result.successful) {
      RCLCPP_ERROR(this->get_logger(), "Failed to set parameter: %s", result.reason.c_str());
    }
  }

  set_param_timer_->cancel();
  param_change_pending_ = false;
}

void InputMapper::set_roll_override(bool roll_override)
{
  // Value hasn't changed, return
  if (roll_override == roll_override_) {
    return;
  }
  // Previous parameter request is still pending, return
  if (param_change_pending_) {
    return;
  }
  roll_override_ = roll_override;

  // Contruct the parameter request object
  auto parameter = rcl_interfaces::msg::Parameter();
  parameter.name = "roll_command_override";
  parameter.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_BOOL;
  parameter.value.bool_value = roll_override;
  set_param_request_ = std::make_shared<rcl_interfaces::srv::SetParameters::Request>();
  set_param_request_->parameters.push_back(parameter);
  set_param_timer_->reset();
  param_change_pending_ = true;
}

void InputMapper::set_pitch_override(bool pitch_override)
{
  // Value hasn't changed, return
  if (pitch_override == pitch_override_) {
    return;
  }
  // Previous parameter request is still pending, return
  if (param_change_pending_) {
    return;
  }
  pitch_override_ = pitch_override;

  // Contruct the parameter request object
  auto parameter = rcl_interfaces::msg::Parameter();
  parameter.name = "pitch_command_override";
  parameter.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_BOOL;
  parameter.value.bool_value = pitch_override;
  set_param_request_ = std::make_shared<rcl_interfaces::srv::SetParameters::Request>();
  set_param_request_->parameters.push_back(parameter);
  set_param_timer_->reset();
  param_change_pending_ = true;
}

double InputMapper::apply_deadzone(double input)
{
  if (abs(input) <= params_.get_double("deadzone_size")) {
    return 0.0;
  } else {
    return input;
  }
}

double InputMapper::clamp_command_to_state(double command, double state, double max_command_diff)
{
  if (abs(command - state) > max_command_diff) {
    if (command > state) {
      return state + max_command_diff;
    } else {
      return state - max_command_diff;
    }
  } else {
    return command;
  }
}

void InputMapper::controller_commands_callback(
  const rosplane_msgs::msg::ControllerCommands::SharedPtr msg)
{
  std::string aileron_input = params_.get_string("aileron_input");
  std::string elevator_input = params_.get_string("elevator_input");
  std::string throttle_input = params_.get_string("throttle_input");
  std::string rudder_input = params_.get_string("rudder_input");

  double elapsed_time = (this->now() - last_command_time_).seconds();
  last_command_time_ = this->now();

  double norm_aileron = (rc_raw_msg_->values[0] - 1500) / 500.0;
  double norm_elevator = (rc_raw_msg_->values[1] - 1500) / 500.0;
  double norm_throttle = (rc_raw_msg_->values[2] - 1500) / 500.0;

  mapped_controller_commands_msg_->header.stamp = this->now();
  mapped_controller_commands_msg_->phi_ff = msg->phi_ff;

  // Aileron channel
  if (aileron_input == "path_follower") {
    set_roll_override(false);
    mapped_controller_commands_msg_->chi_c = msg->chi_c;
  } else if (aileron_input == "rc_course") {
    set_roll_override(false);
    norm_aileron = apply_deadzone(norm_aileron);
    // Apply the rate of change
    mapped_controller_commands_msg_->chi_c +=
      norm_aileron * params_.get_double("rc_course_rate") * elapsed_time;
    // Limit the max difference between state and command
    mapped_controller_commands_msg_->chi_c =
      clamp_command_to_state(mapped_controller_commands_msg_->chi_c, state_msg_->chi,
                             params_.get_double("max_course_diff_command"));
    // Wrap the command within +-180 degrees
    mapped_controller_commands_msg_->chi_c = mapped_controller_commands_msg_->chi_c
      - floor((mapped_controller_commands_msg_->chi_c - state_msg_->chi) / (2 * M_PI) + 0.5) * 2
        * M_PI;
  } else if (aileron_input == "rc_roll_angle") {
    set_roll_override(true);
    mapped_controller_commands_msg_->phi_c =
      norm_aileron * params_.get_double("rc_roll_angle_min_max");
    mapped_controller_commands_msg_->chi_c = state_msg_->chi;
  } else if (aileron_input == "rc_aileron") {
    mapped_controller_commands_msg_->chi_c = state_msg_->chi;
  } else {
    RCLCPP_ERROR(this->get_logger(),
                 "Invalid aileron input type: %s. Valid options are "
                 "path_follower, rc_course, rc_roll_angle, and rc_aileron."
                 "Setting to path_follower.",
                 aileron_input.c_str());
    params_.set_string("aileron_input", "path_follower");
  }

  // Elevator channel
  if (elevator_input == "path_follower") {
    set_pitch_override(false);
    mapped_controller_commands_msg_->h_c = msg->h_c;
  } else if (elevator_input == "rc_altitude") {
    set_pitch_override(false);
    norm_elevator = apply_deadzone(norm_elevator);
    // Apply the rate of change
    mapped_controller_commands_msg_->h_c +=
      norm_elevator * params_.get_double("rc_altitude_rate") * elapsed_time;
    // Limit the max difference between state and command
    mapped_controller_commands_msg_->h_c =
      clamp_command_to_state(mapped_controller_commands_msg_->h_c, -state_msg_->position[2],
                             params_.get_double("max_altitude_diff_command"));
  } else if (elevator_input == "rc_pitch_angle") {
    set_pitch_override(true);
    mapped_controller_commands_msg_->theta_c =
      norm_elevator * params_.get_double("rc_pitch_angle_min_max");
    mapped_controller_commands_msg_->h_c = -state_msg_->position[2];
  } else if (elevator_input == "rc_elevator") {
    mapped_controller_commands_msg_->h_c = -state_msg_->position[2];
  } else {
    RCLCPP_ERROR(this->get_logger(),
                 "Invalid elevator input type: %s. Valid options are "
                 "path_follower, rc_altitude, rc_pitch_angle, and rc_elevator. "
                 "Setting to path_follower.",
                 elevator_input.c_str());
    params_.set_string("elevator_input", "path_follower");
  }

  // Throttle channel
  if (throttle_input == "path_follower") {
    mapped_controller_commands_msg_->va_c = msg->va_c;
  } else if (throttle_input == "rc_airspeed") {
    norm_throttle = apply_deadzone(norm_throttle);
    // Apply the rate of change
    mapped_controller_commands_msg_->va_c +=
      norm_throttle * params_.get_double("rc_airspeed_rate") * elapsed_time;
    // Limit the max difference between state and command
    mapped_controller_commands_msg_->va_c =
      clamp_command_to_state(mapped_controller_commands_msg_->va_c, state_msg_->va,
                             params_.get_double("max_airspeed_diff_command"));
  } else if (throttle_input == "rc_throttle") {
    mapped_controller_commands_msg_->va_c = state_msg_->va;
  } else {
    RCLCPP_ERROR(this->get_logger(),
                 "Invalid throttle input type: %s. Valid options are "
                 "path_follower, rc_airspeed and rc_throttle. Setting to "
                 "path_follower.",
                 throttle_input.c_str());
    params_.set_string("throttle_input", "path_follower");
  }

  // Rudder channel
  if (rudder_input == "yaw_damper") {
  } else if (rudder_input == "rc_rudder") {
  } else {
    RCLCPP_ERROR(this->get_logger(),
                 "Invalid rudder input type: %s. Valid options are "
                 "yaw_damper and rc_rudder. Setting to yaw_damper.",
                 rudder_input.c_str());
    params_.set_string("rudder_input", "yaw_damper");
  }

  mapped_controller_commands_pub_->publish(*mapped_controller_commands_msg_);
}

void InputMapper::command_callback(const rosflight_msgs::msg::Command::SharedPtr msg)
{
  u_int8_t ignore = rosflight_msgs::msg::Command::IGNORE_NONE;
  if (params_.get_string("aileron_input") == "rc_aileron") {
    ignore |= rosflight_msgs::msg::Command::IGNORE_X;
  }
  if (params_.get_string("elevator_input") == "rc_elevator") {
    ignore |= rosflight_msgs::msg::Command::IGNORE_Y;
  }
  if (params_.get_string("rudder_input") == "rc_rudder") {
    ignore |= rosflight_msgs::msg::Command::IGNORE_Z;
  }
  if (params_.get_string("throttle_input") == "rc_throttle") {
    ignore |= rosflight_msgs::msg::Command::IGNORE_F;
  }
  msg->ignore = ignore;
  mapped_command_pub_->publish(*msg);
}

void InputMapper::rc_raw_callback(const rosflight_msgs::msg::RCRaw::SharedPtr msg)
{
  rc_raw_msg_ = msg;
}

void InputMapper::state_callback(const rosplane_msgs::msg::State::SharedPtr msg)
{
  state_msg_ = msg;
}

void InputMapper::path_follower_mode_callback(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  params_.set_string("aileron_input", "path_follower");
  params_.set_string("elevator_input", "path_follower");
  params_.set_string("throttle_input", "path_follower");
  params_.set_string("rudder_input", "yaw_damper");
  response->success = true;
}

void InputMapper::altitude_course_airspeed_control_mode_callback(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  params_.set_string("aileron_input", "rc_course");
  params_.set_string("elevator_input", "rc_altitude");
  params_.set_string("throttle_input", "rc_airspeed");
  params_.set_string("rudder_input", "yaw_damper");
  response->success = true;
}

void InputMapper::angle_control_mode_callback(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  params_.set_string("aileron_input", "rc_roll_angle");
  params_.set_string("elevator_input", "rc_pitch_angle");
  params_.set_string("throttle_input", "rc_throttle");
  params_.set_string("rudder_input", "yaw_damper");
  response->success = true;
}

void InputMapper::rc_passthrough_mode_callback(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  params_.set_string("aileron_input", "rc_aileron");
  params_.set_string("elevator_input", "rc_elevator");
  params_.set_string("throttle_input", "rc_throttle");
  params_.set_string("rudder_input", "rc_rudder");
  response->success = true;
}

rcl_interfaces::msg::SetParametersResult
InputMapper::parametersCallback(const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";

  bool success = params_.set_parameters_callback(parameters);
  if (!success) {
    result.successful = false;
    result.reason =
      "One of the parameters given does not is not a parameter of the input mapper node.";
  }

  return result;
}
} // namespace rosplane

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rosplane::InputMapper>();
  rclcpp::spin(node);
  return 0;
}
