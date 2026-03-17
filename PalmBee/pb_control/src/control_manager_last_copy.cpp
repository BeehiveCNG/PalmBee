#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <mavros_msgs/msg/home_position.hpp>
#include <mavros_msgs/srv/waypoint_push.hpp>
#include <mavros_msgs/msg/waypoint.hpp>
#include <rclcpp/qos.hpp>
#include <cmath>

using namespace std::chrono_literals;

class ControlManager : public rclcpp::Node
{
public:
  ControlManager() : Node("control_manager")
  {
    // MAVROS publishes BEST_EFFORT → must use SensorDataQoS
    gps_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
      "/mavros/global_position/global",
      rclcpp::SensorDataQoS(),
      std::bind(&ControlManager::gps_cb, this, std::placeholders::_1));

    home_sub_ = create_subscription<mavros_msgs::msg::HomePosition>(
      "/mavros/home_position/home",
      rclcpp::SensorDataQoS(),
      std::bind(&ControlManager::home_cb, this, std::placeholders::_1));

    wp_push_ = create_client<mavros_msgs::srv::WaypointPush>("/mavros/mission/push");

    timer_ = create_wall_timer(500ms, std::bind(&ControlManager::run, this));

    RCLCPP_INFO(get_logger(), "GEO-GLOBAL CONTROL MANAGER READY");
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
  rclcpp::Subscription<mavros_msgs::msg::HomePosition>::SharedPtr home_sub_;
  rclcpp::Client<mavros_msgs::srv::WaypointPush>::SharedPtr wp_push_;
  rclcpp::TimerBase::SharedPtr timer_;

  sensor_msgs::msg::NavSatFix gps_;
  mavros_msgs::msg::HomePosition home_;

  bool gps_ok_{false};
  bool home_ok_{false};
  bool sent_{false};

  void gps_cb(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
  {
    if (msg->latitude != 0.0 && msg->longitude != 0.0) {
      gps_ = *msg;
      gps_ok_ = true;
    }
  }

  void home_cb(const mavros_msgs::msg::HomePosition::SharedPtr msg)
  {
    home_ = *msg;
    home_ok_ = true;
  }

  double meters_to_lon(double m, double lat)
  {
    return m / (111320.0 * cos(lat * M_PI / 180.0));
  }

  void run()
  {
    if (!gps_ok_ || !home_ok_ || sent_)
      return;

    double lat = gps_.latitude;
    double lon = gps_.longitude;
    double alt = gps_.altitude;   // relative to EKF home

    double lon2 = lon + meters_to_lon(4.0, lat); // 4 meter east

    auto req = std::make_shared<mavros_msgs::srv::WaypointPush::Request>();

    mavros_msgs::msg::Waypoint wp0, wp1;

    wp0.frame = 3;   // GLOBAL_RELATIVE_ALT
    wp0.command = 16;
    wp0.is_current = true;
    wp0.autocontinue = true;
    wp0.x_lat = lat;
    wp0.y_long = lon;
    wp0.z_alt = alt;

    wp1.frame = 3;
    wp1.command = 16;
    wp1.autocontinue = true;
    wp1.x_lat = lat;
    wp1.y_long = lon2;
    wp1.z_alt = alt;

    req->waypoints = {wp0, wp1};
    wp_push_->async_send_request(req);

    RCLCPP_INFO(get_logger(),
      "Mission uploaded: from (%.8f, %.8f) to (%.8f, %.8f)",
      lat, lon, lat, lon2);

    sent_ = true;
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlManager>());
  rclcpp::shutdown();
}
