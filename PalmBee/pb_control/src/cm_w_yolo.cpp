#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <std_msgs/msg/bool.hpp>

#include <mavros_msgs/srv/waypoint_push.hpp>
#include <mavros_msgs/srv/waypoint_clear.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/command_home.hpp>

#include <mavros_msgs/msg/waypoint.hpp>
#include <mavros_msgs/msg/waypoint_list.hpp>

#include <cmath>

using namespace std::chrono_literals;

enum class State {
  WAIT_GPS,
  SET_HOME,
  ARM,
  UPLOAD_FORWARD,
  AUTO_FORWARD,
  WAIT_INTERRUPT,
  BRAKE,
  WAIT_LOITER_UPLOAD,
  AUTO_LOITER,
  WAIT_LOITER_DONE,
  BRAKE_2,
  WAIT_RESUME_UPLOAD,
  AUTO_RESUME,
  DONE
};

class MissionFSM : public rclcpp::Node
{
public:
  MissionFSM() : Node("mission_fsm")
  {
    gps_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
      "/mavros/global_position/global",
      rclcpp::QoS(10).best_effort(),
      std::bind(&MissionFSM::gps_cb, this, std::placeholders::_1));

    wp_list_sub_ = create_subscription<mavros_msgs::msg::WaypointList>(
      "/mavros/mission/waypoints", 10,
      std::bind(&MissionFSM::wp_list_cb, this, std::placeholders::_1));

    interrupt_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/mission_interrupt", 10,
      std::bind(&MissionFSM::interrupt_cb, this, std::placeholders::_1));

    wp_clear_   = create_client<mavros_msgs::srv::WaypointClear>("/mavros/mission/clear");
    wp_push_    = create_client<mavros_msgs::srv::WaypointPush>("/mavros/mission/push");
    mode_client_= create_client<mavros_msgs::srv::SetMode>("/mavros/set_mode");
    arm_client_ = create_client<mavros_msgs::srv::CommandBool>("/mavros/cmd/arming");
    home_client_= create_client<mavros_msgs::srv::CommandHome>("/mavros/cmd/set_home");

    timer_ = create_wall_timer(500ms, std::bind(&MissionFSM::fsm, this));

    RCLCPP_INFO(get_logger(), "AUTO Mission FSM (with interrupt) READY");
  }

private:
  // ROS
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
  rclcpp::Subscription<mavros_msgs::msg::WaypointList>::SharedPtr wp_list_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr interrupt_sub_;

  rclcpp::Client<mavros_msgs::srv::WaypointClear>::SharedPtr wp_clear_;
  rclcpp::Client<mavros_msgs::srv::WaypointPush>::SharedPtr wp_push_;
  rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr mode_client_;
  rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arm_client_;
  rclcpp::Client<mavros_msgs::srv::CommandHome>::SharedPtr home_client_;
  rclcpp::TimerBase::SharedPtr timer_;

  // State
  State state_{State::WAIT_GPS};
  sensor_msgs::msg::NavSatFix home_gps_;
  bool gps_ready_{false};
  bool interrupt_{false};
  int wp_count_{0};
  int current_seq_{0};

  // ========================
  void gps_cb(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
  {
    home_gps_ = *msg;
    gps_ready_ = true;
  }

  void wp_list_cb(const mavros_msgs::msg::WaypointList::SharedPtr msg)
  {
    wp_count_ = msg->waypoints.size();
    current_seq_ = msg->current_seq;
  }

  void interrupt_cb(const std_msgs::msg::Bool::SharedPtr msg)
  {
    interrupt_ = msg->data;
  }

  // ========================
  void fsm()
  {
    switch (state_)
    {
    case State::WAIT_GPS:
      if (gps_ready_) {
        set_home();
        state_ = State::SET_HOME;
        RCLCPP_INFO(get_logger(), "GPS ready → SET HOME");
      }
      break;

    case State::SET_HOME:
      arm();
      state_ = State::ARM;
      RCLCPP_INFO(get_logger(), "ARM");
      break;

    case State::ARM:
      upload_forward();
      state_ = State::UPLOAD_FORWARD;
      RCLCPP_INFO(get_logger(), "UPLOAD FORWARD");
      break;

    case State::UPLOAD_FORWARD:
      if (wp_count_ == 2) {
        change_mode("AUTO");
        state_ = State::AUTO_FORWARD;
        RCLCPP_INFO(get_logger(), "AUTO FORWARD");
      }
      break;

    case State::AUTO_FORWARD:
      state_ = State::WAIT_INTERRUPT;
      break;

    case State::WAIT_INTERRUPT:
      if (interrupt_) {
        change_mode("BRAKE");
        state_ = State::BRAKE;
        RCLCPP_INFO(get_logger(), "INTERRUPT → BRAKE");
      }
      break;

    case State::BRAKE:
      upload_loiter();
      state_ = State::WAIT_LOITER_UPLOAD;
      RCLCPP_INFO(get_logger(), "UPLOAD LOITER");
      break;

    case State::WAIT_LOITER_UPLOAD:
      if (wp_count_ == 1) {
        change_mode("AUTO");
        state_ = State::AUTO_LOITER;
        RCLCPP_INFO(get_logger(), "AUTO LOITER");
      }
      break;

    case State::AUTO_LOITER:
      state_ = State::WAIT_LOITER_DONE;
      break;

    case State::WAIT_LOITER_DONE:
      if (current_seq_ > 0) {
        change_mode("BRAKE");
        state_ = State::BRAKE_2;
        RCLCPP_INFO(get_logger(), "LOITER DONE → BRAKE");
      }
      break;

    case State::BRAKE_2:
      upload_resume();
      state_ = State::WAIT_RESUME_UPLOAD;
      RCLCPP_INFO(get_logger(), "UPLOAD RESUME");
      break;

    case State::WAIT_RESUME_UPLOAD:
      if (wp_count_ == 2) {
        change_mode("AUTO");
        state_ = State::AUTO_RESUME;
        RCLCPP_INFO(get_logger(), "AUTO RESUME");
      }
      break;

    case State::AUTO_RESUME:
      state_ = State::DONE;
      RCLCPP_INFO(get_logger(), "MISSION DONE");
      break;

    case State::DONE:
      break;
    }
  }

  // ========================
  void set_home()
  {
    auto req = std::make_shared<mavros_msgs::srv::CommandHome::Request>();
    req->current_gps = true;
    home_client_->async_send_request(req);
  }

  void arm()
  {
    auto req = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
    req->value = true;
    arm_client_->async_send_request(req);
  }

  // ========================
  void upload_forward()
  {
    double lat = home_gps_.latitude;
    double lon = home_gps_.longitude;

    double dlon = 6.0 / (111320.0 * cos(lat * M_PI / 180.0));
    double target_lon = lon + dlon;

    wp_clear_->async_send_request(
      std::make_shared<mavros_msgs::srv::WaypointClear::Request>());

    auto req = std::make_shared<mavros_msgs::srv::WaypointPush::Request>();

    mavros_msgs::msg::Waypoint wp0;
    wp0.frame = 3;
    wp0.command = 22;
    wp0.is_current = true;
    wp0.autocontinue = true;
    wp0.x_lat = lat;
    wp0.y_long = lon;
    wp0.z_alt = 2.0;

    mavros_msgs::msg::Waypoint wp1;
    wp1.frame = 3;
    wp1.command = 16;
    wp1.autocontinue = true;
    wp1.x_lat = lat;
    wp1.y_long = target_lon;
    wp1.z_alt = 2.0;

    req->waypoints = {wp0, wp1};
    wp_push_->async_send_request(req);
  }

  // ========================
  void upload_loiter()
  {
    double lat = home_gps_.latitude;
    double lon = home_gps_.longitude;

    double dlon = 3.0 / (111320.0 * cos(lat * M_PI / 180.0));
    double dlat = -2.0 / 111320.0;

    wp_clear_->async_send_request(
      std::make_shared<mavros_msgs::srv::WaypointClear::Request>());

    auto req = std::make_shared<mavros_msgs::srv::WaypointPush::Request>();

    mavros_msgs::msg::Waypoint wp0;
    wp0.frame = 3;
    wp0.command = 18;
    wp0.is_current = true;
    wp0.autocontinue = true;
    wp0.param1 = 2;    // 2 turns
    wp0.param2 = 0;    // CW
    wp0.param3 = 2.0;  // radius 2m
    wp0.x_lat = lat + dlat;
    wp0.y_long = lon + dlon;
    wp0.z_alt = 2.0;

    req->waypoints = {wp0};
    wp_push_->async_send_request(req);
  }

  // ========================
  void upload_resume()
  {
    double lat = home_gps_.latitude;
    double lon = home_gps_.longitude;

    double dlon = 6.0 / (111320.0 * cos(lat * M_PI / 180.0));
    double target_lon = lon + dlon;

    wp_clear_->async_send_request(
      std::make_shared<mavros_msgs::srv::WaypointClear::Request>());

    auto req = std::make_shared<mavros_msgs::srv::WaypointPush::Request>();

    mavros_msgs::msg::Waypoint wp0;
    wp0.frame = 3;
    wp0.command = 16;
    wp0.is_current = true;
    wp0.autocontinue = true;
    wp0.x_lat = lat;
    wp0.y_long = target_lon;
    wp0.z_alt = 2.0;

    mavros_msgs::msg::Waypoint wp1;
    wp1.frame = 3;
    wp1.command = 21;
    wp1.autocontinue = true;
    wp1.x_lat = lat;
    wp1.y_long = target_lon;
    wp1.z_alt = 0;

    req->waypoints = {wp0, wp1};
    wp_push_->async_send_request(req);
  }

  // ========================
  void change_mode(const std::string &mode)
  {
    auto req = std::make_shared<mavros_msgs::srv::SetMode::Request>();
    req->custom_mode = mode;
    mode_client_->async_send_request(req);
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MissionFSM>());
  rclcpp::shutdown();
  return 0;
}
