#include <memory>
#include <vector>
#include <cmath>

#include "rclcpp/rclcpp.hpp"

#include "mavros_msgs/msg/command_code.hpp"
#include "mavros_msgs/srv/waypoint_clear.hpp"
#include "mavros_msgs/srv/waypoint_push.hpp"
#include "mavros_msgs/msg/waypoint.hpp"

#include "geographic_msgs/msg/geo_point_stamped.hpp"

using std::placeholders::_1;

// MAVLink numeric frame (portable, Humble-safe)
static constexpr uint8_t MAV_FRAME_GLOBAL_RELATIVE_ALT = 3;

class EKFOriginMissionUploader : public rclcpp::Node
{
public:
  EKFOriginMissionUploader()
  : Node("ekf_origin_mission_uploader"),
    origin_received_(false)
  {
    auto qos = rclcpp::QoS(rclcpp::KeepLast(1))
                 .reliable()
                 .transient_local();

    origin_sub_ = this->create_subscription<
      geographic_msgs::msg::GeoPointStamped>(
        "/mavros/global_position/gp_origin",
        qos,
        std::bind(&EKFOriginMissionUploader::originCallback, this, _1)
      );

    clear_client_ = this->create_client<mavros_msgs::srv::WaypointClear>(
      "/mavros/mission/clear"
    );

    push_client_ = this->create_client<mavros_msgs::srv::WaypointPush>(
      "/mavros/mission/push"
    );

    RCLCPP_INFO(this->get_logger(), "Waiting for EKF origin...");
  }

private:
  // ===== EKF ORIGIN =====
  bool   origin_received_;
  double origin_lat_;
  double origin_lon_;
  double origin_alt_;

  void originCallback(
    const geographic_msgs::msg::GeoPointStamped::SharedPtr msg)
  {
    if (origin_received_) return;

    origin_received_ = true;

    origin_lat_ = msg->position.latitude;
    origin_lon_ = msg->position.longitude;
    origin_alt_ = msg->position.altitude;

    RCLCPP_INFO(
      this->get_logger(),
      "EKF origin received (lat=%.8f, lon=%.8f, alt=%.2f)",
      origin_lat_, origin_lon_, origin_alt_
    );

    uploadMission();
  }

  // ===== METERS → LAT/LON =====
  void metersToLatLon(
    double north_m,
    double east_m,
    double &lat_out,
    double &lon_out)
  {
    constexpr double R = 6378137.0; // Earth radius (WGS84)
    double lat_rad = origin_lat_ * M_PI / 180.0;

    lat_out = origin_lat_ + (north_m / R) * 180.0 / M_PI;
    lon_out = origin_lon_ + (east_m  / (R * cos(lat_rad))) * 180.0 / M_PI;
  }

  // ===== MISSION =====
  void uploadMission()
  {
    const float takeoff_alt = 1.5f;
    const float move_dist   = 2.0f;
    const float loiter_time = 2.0f;

    std::vector<mavros_msgs::msg::Waypoint> waypoints;

    double lat, lon;

    // WP0 — climb at EKF origin
    {
      mavros_msgs::msg::Waypoint wp;
      wp.frame = MAV_FRAME_GLOBAL_RELATIVE_ALT;
      wp.command = mavros_msgs::msg::CommandCode::NAV_WAYPOINT;
      wp.is_current = true;
      wp.autocontinue = true;
      wp.x_lat  = origin_lat_;
      wp.y_long = origin_lon_;
      wp.z_alt  = takeoff_alt;
      waypoints.push_back(wp);
    }

    // WP1 — move FORWARD (EAST, because yaw ≈ 90°)
    metersToLatLon(0.0, move_dist, lat, lon);

    {
      mavros_msgs::msg::Waypoint wp;
      wp.frame = MAV_FRAME_GLOBAL_RELATIVE_ALT;
      wp.command = mavros_msgs::msg::CommandCode::NAV_WAYPOINT;
      wp.autocontinue = true;
      wp.x_lat  = lat;
      wp.y_long = lon;
      wp.z_alt  = takeoff_alt;
      waypoints.push_back(wp);
    }

    // WP2 — loiter
    {
      mavros_msgs::msg::Waypoint wp;
      wp.frame = MAV_FRAME_GLOBAL_RELATIVE_ALT;
      wp.command = mavros_msgs::msg::CommandCode::NAV_LOITER_TIME;
      wp.autocontinue = true;
      wp.param1 = loiter_time;
      wp.x_lat  = lat;
      wp.y_long = lon;
      wp.z_alt  = takeoff_alt;
      waypoints.push_back(wp);
    }

    // WP3 — land
    {
      mavros_msgs::msg::Waypoint wp;
      wp.frame = MAV_FRAME_GLOBAL_RELATIVE_ALT;
      wp.command = mavros_msgs::msg::CommandCode::NAV_LAND;
      wp.autocontinue = true;
      wp.x_lat  = lat;
      wp.y_long = lon;
      wp.z_alt  = 0.0;
      waypoints.push_back(wp);
    }

    // CLEAR → PUSH (async, no wait)
    auto clear_req =
      std::make_shared<mavros_msgs::srv::WaypointClear::Request>();

    clear_client_->async_send_request(
      clear_req,
      [this, waypoints](
        rclcpp::Client<mavros_msgs::srv::WaypointClear>::SharedFuture)
      {
        RCLCPP_INFO(
          this->get_logger(),
          "Mission cleared, pushing GLOBAL_RELATIVE_ALT mission"
        );
        pushMission(waypoints);
      }
    );
  }

  void pushMission(
    const std::vector<mavros_msgs::msg::Waypoint>& waypoints)
  {
    auto push_req =
      std::make_shared<mavros_msgs::srv::WaypointPush::Request>();
    push_req->waypoints = waypoints;

    push_client_->async_send_request(
      push_req,
      std::bind(
        &EKFOriginMissionUploader::uploadResult,
        this,
        std::placeholders::_1
      )
    );
  }

  void uploadResult(
    rclcpp::Client<mavros_msgs::srv::WaypointPush>::SharedFuture future)
  {
    auto result = future.get();

    if (result->success) {
      RCLCPP_INFO(
        this->get_logger(),
        "Mission uploaded successfully (%u waypoints)",
        result->wp_transfered
      );
    } else {
      RCLCPP_ERROR(this->get_logger(), "Mission upload FAILED");
    }
  }

  // ROS interfaces
  rclcpp::Subscription<
    geographic_msgs::msg::GeoPointStamped>::SharedPtr origin_sub_;
  rclcpp::Client<mavros_msgs::srv::WaypointClear>::SharedPtr clear_client_;
  rclcpp::Client<mavros_msgs::srv::WaypointPush>::SharedPtr push_client_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EKFOriginMissionUploader>());
  rclcpp::shutdown();
  return 0;
}
