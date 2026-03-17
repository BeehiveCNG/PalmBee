#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/msg/position_target.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <mavros_msgs/srv/command_bool.hpp>

#include <cmath>
#include <chrono>

using namespace std::chrono_literals;

enum class FlightState {
  WAIT_CONNECTION,
  WAIT_POSE,
  PRESTREAM,
  SET_GUIDED,
  ARM,
  STABILIZE_AFTER_ARM,
  TAKEOFF_RAMP,
  HOVER,
  MOVE_FORWARD,
  LAND,
  HOLD
};

class GuidedControlRaw : public rclcpp::Node {
public:
  GuidedControlRaw() : Node("guided_control_raw")
  {
    auto sensor_qos =
        rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();

    pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        "/mavros/local_position/pose",
        sensor_qos,
        std::bind(&GuidedControlRaw::pose_cb,this,std::placeholders::_1));

    vel_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
        "/mavros/local_position/velocity_local",
        sensor_qos,
        std::bind(&GuidedControlRaw::vel_cb,this,std::placeholders::_1));

    state_sub_ = create_subscription<mavros_msgs::msg::State>(
        "/mavros/state",10,
        std::bind(&GuidedControlRaw::state_cb,this,std::placeholders::_1));

    setpoint_pub_ =
      create_publisher<mavros_msgs::msg::PositionTarget>(
          "/mavros/setpoint_raw/local",10);

    mode_client_ =
      create_client<mavros_msgs::srv::SetMode>("/mavros/set_mode");

    arm_client_ =
      create_client<mavros_msgs::srv::CommandBool>("/mavros/cmd/arming");

    timer_ = create_wall_timer(50ms,
        std::bind(&GuidedControlRaw::fsm_step,this));

    RCLCPP_INFO(get_logger(),"🔥 PRO Indoor FSM v4 started");
  }

private:

  /* ===== CONFIG ===== */
  const double takeoff_alt_ = 1.5;
  const double takeoff_rate_ = 0.02;   // m per cycle
  const double forward_dist_ = 2.0;
  const double pos_tol_ = 0.15;
  const double vel_tol_ = 0.15;

  /* ===== STATE ===== */
  FlightState state_{FlightState::WAIT_CONNECTION};
  rclcpp::Time state_start_;

  mavros_msgs::msg::State current_state_;
  geometry_msgs::msg::PoseStamped current_pose_;
  geometry_msgs::msg::TwistStamped current_vel_;

  mavros_msgs::msg::PositionTarget target_;

  bool pose_ready_{false};
  rclcpp::Time last_pose_time_;

  double start_z_{0.0};
  double forward_target_x_{0.0};

  /* ===== ROS ===== */
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr vel_sub_;
  rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
  rclcpp::Publisher<mavros_msgs::msg::PositionTarget>::SharedPtr setpoint_pub_;
  rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr mode_client_;
  rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arm_client_;
  rclcpp::TimerBase::SharedPtr timer_;

  /* ===== CALLBACKS ===== */

  void pose_cb(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    current_pose_ = *msg;
    last_pose_time_ = now();

    if(!pose_ready_) {
      pose_ready_ = true;
      target_.position = current_pose_.pose.position;
      target_.yaw = 0.0;
      RCLCPP_INFO(get_logger(),"Local pose ready");
    }
  }

  void vel_cb(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
  { current_vel_ = *msg; }

  void state_cb(const mavros_msgs::msg::State::SharedPtr msg)
  { current_state_ = *msg; }

  /* ===== UTILS ===== */

  bool elapsed(double s){
    return (now()-state_start_).seconds() > s;
  }

  void transition(FlightState s){
    state_ = s;
    state_start_ = now();
    RCLCPP_INFO(get_logger(),"STATE -> %d",(int)s);
  }

  double vel_norm(){
    return std::sqrt(
      std::pow(current_vel_.twist.linear.x,2)+
      std::pow(current_vel_.twist.linear.y,2)+
      std::pow(current_vel_.twist.linear.z,2));
  }

  bool vision_ok(){
    return (now()-last_pose_time_).seconds() < 0.3;
  }

  void publish_setpoint()
  {
    target_.header.stamp = now();
    target_.coordinate_frame =
      mavros_msgs::msg::PositionTarget::FRAME_LOCAL_NED;

    // ⭐ ARDUPILOT-CORRECT MASK (2552)
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
    req->custom_mode = "GUIDED_NOGPS";
    mode_client_->async_send_request(req);
  }

  void arm(){
    auto req = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
    req->value = true;
    arm_client_->async_send_request(req);
  }

  /* ===== FSM ===== */

  void fsm_step()
  {
    publish_setpoint();

    // Vision safety
    if(pose_ready_ && !vision_ok()){
      transition(FlightState::HOLD);
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
        if(current_state_.mode != "GUIDED_NOGPS")
          set_guided();
        else
          transition(FlightState::ARM);
        break;

      case FlightState::ARM:
        if(!current_state_.armed)
          arm();
        else {
          start_z_ = current_pose_.pose.position.z;
          transition(FlightState::STABILIZE_AFTER_ARM);
        }
        break;

      case FlightState::STABILIZE_AFTER_ARM:
        if(elapsed(2.0))
          transition(FlightState::TAKEOFF_RAMP);
        break;

      case FlightState::TAKEOFF_RAMP:
      {
        double target_z = start_z_ - takeoff_alt_; // NED!

        if(target_.position.z > target_z)
          target_.position.z -= takeoff_rate_;

        if(std::abs(current_pose_.pose.position.z - target_z) < pos_tol_)
          transition(FlightState::HOVER);
        break;
      }

      case FlightState::HOVER:
        if(elapsed(3.0)) {
          forward_target_x_ =
              current_pose_.pose.position.x + forward_dist_;
          transition(FlightState::MOVE_FORWARD);
        }
        break;

      case FlightState::MOVE_FORWARD:
        target_.position.x = forward_target_x_;
        break;

      case FlightState::LAND:
        target_.position.z += 0.02;
        break;

      case FlightState::HOLD:
        target_.position = current_pose_.pose.position;
        break;
    }
  }
};

int main(int argc,char** argv)
{
  rclcpp::init(argc,argv);
  rclcpp::spin(std::make_shared<GuidedControlRaw>());
  rclcpp::shutdown();
  return 0;
}
