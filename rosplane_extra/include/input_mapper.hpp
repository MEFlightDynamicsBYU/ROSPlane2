#ifndef INPUT_MAPPER_HPP
#define INPUT_MAPPER_HPP

#include <rclcpp/rclcpp.hpp>
#include <rosflight_msgs/msg/command.hpp>
#include <rosflight_msgs/msg/rc_raw.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "param_manager/param_manager.hpp"
#include "rosplane_msgs/msg/controller_commands.hpp"
#include "rosplane_msgs/msg/state.hpp"

namespace rosplane
{
class InputMapper : public rclcpp::Node
{
public:
  /**
   * @class InputMapper
   * @brief Class for input mixing.
   *
   * The input_mapper class is responsible for mixing various input commands, such as controller
   * commands and RC raw signals.
   *
   * This node has the following ROS parameters:
   * Mapping Options:
   * - `aileron_input`: The type of input to use for the aileron channel. Options are
   *   "path_follower", "rc_course", "rc_roll_angle", and "rc_aileron". Path follower just pipes the
   *   control commands from path follower to the autopilot. RC course uses the RC controller to
   *   adjust course command at a rate specified by `rc_course_rate`. RC roll angle uses the RC
   *   controller to set the commanded roll angle directly. RC aileron is a RC passthrough mode.
   * - `elevator_input`: The type of input to use for the elevator channel. Options are
   *   "path_follower", "rc_altitude", "rc_pitch_angle", and "rc_elevator. Path follower just pipes the
   *   control commands from path follower to the autopilot. RC altitude uses the RC controller
   *   to adjust altitude command at a rate specified by `rc_altitude_rate`. RC pitch angle uses
   *   the RC controller to set the commanded pitch angle directly. RC elevator is an RC passthrough mode.
   * - `throttle_input`: The type of input to use for the throttle channel. Options are
   *   "path_follower", "rc_airspeed", "rc_throttle". Path follower just pipes the control commands from path
   *   follower to the autopilot. RC airspeed uses the RC controller to adjust the airspeed
   *   command at a rate specified by `rc_airspeed_rate`. RC throttle is an RC passthrough mode.
   * - `rudder_input`: The type of input to use for the rudder channel. Options are "yaw_damper" and
   *   "rc_rudder". Yaw damper lets the autopilot control the rudder. RC rudder is an RC passthrough mode.
   *
   * Control Parameters:
   * - `rc_roll_angle_min_max`: The maximum roll angle in radians that can be commanded by the RC
   *   controller. Roll is centered about 0 radians.
   * - `rc_course_rate`: The max rate in radians per second at which the RC controller can adjust
   *   the course command.
   * - `rc_pitch_angle_min_max`: The maximum pitch angle in radians that can be commanded by the RC
   *   controller. Pitch is centered about 0 radians.
   * - `rc_altitude_rate`: The max rate in meters per second at which the RC controller can adjust
   *   the altitude command.
   * - `rc_airspeed_rate`: The max rate in meters per second at which the RC controller can adjust
   *   the airspeed command.
   * - `deadzone_size`: The size of deviations on the RC transmitter to ignore, avoiding gradual
   *   drift of any rate control mode.
   * - `max_course_diff_command`: The maximum allowed difference between the commanded course
   *   and the vehicle's current state, avoiding command runaway.
   * - `max_altitude_diff_command`: The maximum allowed difference between the commanded altitude
   *   and the vehicle's current state, avoiding command runaway.
   * - `max_airspeed_diff_command`: The maximum allowed difference between the commanded airspeed
   *   and the vehicle's current state, avoiding command runaway.
   */
  InputMapper();

private:
  /**
   * This bool keeps track of whether roll override is enabled.
   */
  bool roll_override_;
  /**
   * This bool keeps track of whether pitch override is enabled.
   */
  bool pitch_override_;
  /**
   * Bool value to keep track of whether a parameter change is pending.
   */
  bool param_change_pending_;

  /**
  * Keeps track of previous time of the last controller command sent for rate control.
  */
  rclcpp::Time last_command_time_;

  /**
   * This publisher publishes the mapped controller commands for the autopilot.
   */
  rclcpp::Publisher<rosplane_msgs::msg::ControllerCommands>::SharedPtr
    mapped_controller_commands_pub_;
  /**
   * This publisher publishes the mapped command for the firmware.
   */
  rclcpp::Publisher<rosflight_msgs::msg::Command>::SharedPtr mapped_command_pub_;

  /**
   * This subscriber subscribes to the path follower controller commands.
   */
  rclcpp::Subscription<rosplane_msgs::msg::ControllerCommands>::SharedPtr controller_commands_sub_;
  /**
   * This subscriber subscribes to the commands being sent to the firmware.
   */
  rclcpp::Subscription<rosflight_msgs::msg::Command>::SharedPtr command_sub_;
  /**
   * This subscriber subscribes to the RC raw signals.
   */
  rclcpp::Subscription<rosflight_msgs::msg::RCRaw>::SharedPtr rc_raw_sub_;
  /**
   * This subscriber subscribes to the estimated state, to help make more intelligent
   * decisions/transitions within input_mapper.
   */
  rclcpp::Subscription<rosplane_msgs::msg::State>::SharedPtr state_sub_;

  /**
   * RC raw message, for storing the last RC input before being mapped and published.
   */
  rosflight_msgs::msg::RCRaw::SharedPtr rc_raw_msg_;
  /**
   * Controller commands message, for storing the last commands after being mapped.
   */
  rosplane_msgs::msg::ControllerCommands::SharedPtr mapped_controller_commands_msg_;
  /**
   * State message, for storing the last state message received.
   */
  rosplane_msgs::msg::State::SharedPtr state_msg_;

  /**
   * Service object for setting parameters of the controller.
   */
  rclcpp::Client<rcl_interfaces::srv::SetParameters>::SharedPtr set_param_client_;

  /**
   * Timer for setting parameters of other nodes without ever blocking execution of this node.
   *
   * This timer exists so that waiting for the set param service and response will not block the
   * execution of the node, which needs to keep running to keep passing inputs around.
   */
  rclcpp::TimerBase::SharedPtr set_param_timer_;

  /**
   * Parameter request object, for setting parameters of other nodes.
   */
  rcl_interfaces::srv::SetParameters::Request::SharedPtr set_param_request_;

  /**
   * FutureAndRequestId result for param set request. Used to obtain the result of the request
   * and print errors if the service failed.
   */
  rclcpp::Client<rcl_interfaces::srv::SetParameters>::SharedFuture set_param_future_;

  /**
   * Service for setting input methods to path follower mode.
   */
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr path_follower_mode_service_;
  /**
   * Service for setting input methods to altitude course airspeed control mode.
   */
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr altitude_course_airspeed_control_mode_service_;
  /**
   * Service for setting input methods to angle control mode.
   */
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr angle_control_mode_service_;
  /**
   * Service for setting input methods to RC passthrough mode.
   */
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr rc_passthrough_mode_service_;

  /**
   * Helper function for knowing when to call a ROS service to change roll override.
   */
  void set_roll_override(bool roll_override);
  /**
   * Helper function for knowing when to call a ROS service to change pitch override.
   */
  void set_pitch_override(bool pitch_override);

  /**
   * Helper function for applying deadzones to RC input.
   */
  double apply_deadzone(double input);
  /**
   * Helper function for keeping the state of the aircraft and the command close to each other.
   */
  double clamp_command_to_state(double command, double state, double command_diff);

  /**
   * Callback for set_param_timer_.
   */
  void set_param_timer_callback();

  /**
   * This function is called when a new message of type `rosplane_msgs::msg::ControllerCommands` is
   * received.
   *
   * @param msg A shared pointer to the received message.
   */
  void controller_commands_callback(const rosplane_msgs::msg::ControllerCommands::SharedPtr msg);
  /**
   * This function is called when a new message of type `rosplane_msgs::msg::Command` is received.
   *
   * @param msg A shared pointer to the received message.
   */
  void command_callback(const rosflight_msgs::msg::Command::SharedPtr msg);
  /**
   * This function is called when a new message of type `rosflight_msgs::msg::RCRaw` is received.
   *
   * @param msg A shared pointer to the received message.
   */
  void rc_raw_callback(const rosflight_msgs::msg::RCRaw::SharedPtr msg);
  /**
   * This function is called when a new message of type `rosplane_msgs::msg::State` is received.
   *
   * @param msg A shared pointer to the received message.
   */
  void state_callback(const rosplane_msgs::msg::State::SharedPtr msg);

  /**
   * This function is called when the path follower mode service is called, setting the input
   * modes to path follower mode.
   *
   * @param request The request object for the service.
   * @param response The response object for the service.
   */
  void path_follower_mode_callback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                   std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  /**
   * This function is called when the altitude course airspeed control mode service is called,
   * setting the input modes to altitude, course, and airspeed rate control.
   *
   * @param request The request object for the service.
   * @param response The response object for the service.
   */
  void altitude_course_airspeed_control_mode_callback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  /**
   * This function is called when the angle control mode service is called, setting the input
   * modes to angle control mode.
   *
   * @param request The request object for the service.
   * @param response The response object for the service.
   */
  void angle_control_mode_callback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                   std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  /**
   * This function is called when the RC passthrough mode service is called, setting the input
   * modes to RC passthrough.
   *
   * @param request The request object for the service.
   * @param response The response object for the service.
   */
  void rc_passthrough_mode_callback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                    std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  /// Parameters stuff

  /**
   * Param manager object, for getting parameters from ROS.
   */
  ParamManager params_;

  /**
   * ROS2 parameter system interface. This connects ROS2 parameters with the defined update callback,
   * parametersCallback.
   */
  OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;

  /**
   * Callback for when parameters are changed using ROS2 parameter system.
   * This takes all new changed params and updates the appropriate parameters in the params_ object.
   * @param parameters Set of updated parameters.
   * @return Service result object that tells the requester the result of the param update.
   */
  rcl_interfaces::msg::SetParametersResult
  parametersCallback(const std::vector<rclcpp::Parameter> & parameters);
};
} // namespace rosplane

#endif //INPUT_MAPPER_HPP
