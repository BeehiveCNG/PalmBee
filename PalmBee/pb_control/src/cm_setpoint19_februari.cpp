#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/msg/position_target.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <mavros_msgs/srv/command_bool.hpp>

#include <chrono>
#include <cmath>

using namespace std::chrono_literals;

enum class FlightState {
  WAIT_CONNECTION,
  WAIT_POSE,
  PRESTREAM,
  SET_GUIDED,
  ARM_WAIT,
  STABILIZE_AFTER_ARM,
  TAKEOFF,
  HOVER
};

class FSMv6 : public rclcpp::Node {
public:
  FSMv6() : Node("fsm_v6_fixed_takeoff")
  {
    pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/mavros/local_position/pose",
      rclcpp::SensorDataQoS(),
      std::bind(&FSMv6::pose_cb,this,std::placeholders::_1));

    state_sub_ = create_subscription<mavros_msgs::msg::State>(
      "/mavros/state",10,
      std::bind(&FSMv6::state_cb,this,std::placeholders::_1));

    setpoint_pub_ = create_publisher<mavros_msgs::msg::PositionTarget>(
      "/mavros/setpoint_raw/local",10);

    mode_client_ =
      create_client<mavros_msgs::srv::SetMode>("/mavros/set_mode");

    arm_client_ =
      create_client<mavros_msgs::srv::CommandBool>("/mavros/cmd/arming");

    timer_ = create_wall_timer(50ms,
      std::bind(&FSMv6::fsm_step,this));

    RCLCPP_INFO(get_logger(),"🚀 FSM FIXED (Humble compatible)");
  }

private:

  const double takeoff_alt_ = 1.2;
  const double pos_tol_ = 0.15;

  FlightState state_{FlightState::WAIT_CONNECTION};
  rclcpp::Time state_start_;

  mavros_msgs::msg::State current_state_;
  geometry_msgs::msg::PoseStamped current_pose_;

  mavros_msgs::msg::PositionTarget target_;

  bool pose_ready_{false};
  bool was_armed_{false};
  bool arm_sent_{false};

  double start_z_enu_{0.0};

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
  rclcpp::Publisher<mavros_msgs::msg::PositionTarget>::SharedPtr setpoint_pub_;
  rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr mode_client_;
  rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arm_client_;
  rclcpp::TimerBase::SharedPtr timer_;

  /* ---------- callbacks ---------- */

  void pose_cb(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    current_pose_ = *msg;

    if(!pose_ready_){
      pose_ready_ = true;

      target_.position.x = msg->pose.position.x;
      target_.position.y = msg->pose.position.y;
      target_.position.z = -msg->pose.position.z; // ENU -> NED

      target_.yaw = 0.0;

      RCLCPP_INFO(get_logger(),"Pose ready");
    }
  }

  void state_cb(const mavros_msgs::msg::State::SharedPtr msg)
  {
    current_state_ = *msg;
  }

  bool elapsed(double sec){
    return (now()-state_start_).seconds() > sec;
  }

  void transition(FlightState s){
    state_ = s;
    state_start_ = now();
    RCLCPP_INFO(get_logger(),"STATE -> %d",(int)s);
  }

  void publish_setpoint()
  {
    target_.header.stamp = now();

    /* Humble MAVROS → NED only */
    target_.coordinate_frame =
      mavros_msgs::msg::PositionTarget::FRAME_LOCAL_NED;

    target_.type_mask =
      mavros_msgs::msg::PositionTarget::IGNORE_VX |
      mavros_msgs::msg::PositionTarget::IGNORE_VY |
      mavros_msgs::msg::PositionTarget::IGNORE_VZ |
      mavros_msgs::msg::PositionTarget::IGNORE_AFX |
      mavros_msgs::msg::PositionTarget::IGNORE_AFY |
      mavros_msgs::msg::PositionTarget::IGNORE_AFZ |
      mavros_msgs::msg::PositionTarget::IGNORE_YAW_RATE;

    setpoint_pub_->publish(target_);
  }

  void set_guided(){
    auto req = std::make_shared<mavros_msgs::srv::SetMode::Request>();
    req->custom_mode = "GUIDED";
    mode_client_->async_send_request(req);
  }

  void arm(){
    auto req = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
    req->value = true;
    arm_client_->async_send_request(req);
  }

  /* ---------- FSM ---------- */

  void fsm_step()
  {
    publish_setpoint();

    if(!was_armed_ && current_state_.armed){
      was_armed_ = true;

      start_z_enu_ = current_pose_.pose.position.z;

      target_.position.x = current_pose_.pose.position.x;
      target_.position.y = current_pose_.pose.position.y;
      target_.position.z = -current_pose_.pose.position.z;

      transition(FlightState::STABILIZE_AFTER_ARM);
    }

    switch(state_)
    {
      case FlightState::WAIT_CONNECTION:
        if(current_state_.connected)
          transition(FlightState::WAIT_POSE);
        break;

      case FlightState::WAIT_POSE:
        if(pose_ready_)
          transition(FlightState::PRESTREAM);
        break;

      case FlightState::PRESTREAM:
        if(elapsed(2.0))
          transition(FlightState::SET_GUIDED);
        break;

      case FlightState::SET_GUIDED:
        if(current_state_.mode != "GUIDED"){
          set_guided();
        } else if(elapsed(1.0)){
          transition(FlightState::ARM_WAIT);
        }
        break;

      case FlightState::ARM_WAIT:
        if(!current_state_.armed && !arm_sent_){
          arm();
          arm_sent_ = true;
        }
        break;

      case FlightState::STABILIZE_AFTER_ARM:
        if(elapsed(3.0))
          transition(FlightState::TAKEOFF);
        break;

      case FlightState::TAKEOFF:
      {
        /* ENU up → NED down */
        double target_z_enu = start_z_enu_ + takeoff_alt_;
        target_.position.z = -target_z_enu;

        if(std::abs(current_pose_.pose.position.z - target_z_enu) < pos_tol_)
          transition(FlightState::HOVER);

        break;
      }

      case FlightState::HOVER:
        break;
    }
  }
};

int main(int argc,char** argv)
{
  rclcpp::init(argc,argv);
  rclcpp::spin(std::make_shared<FSMv6>());
  rclcpp::shutdown();
  return 0;
}
