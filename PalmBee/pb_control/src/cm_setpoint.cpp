#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/srv/command_tol.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Matrix3x3.h>

#include <vector>
#include <cmath>

using namespace std::chrono_literals;

class TakeoffManager : public rclcpp::Node
{
public:
TakeoffManager():Node("cm_setpoint")
{

state_sub_=create_subscription<mavros_msgs::msg::State>(
"/mavros/state",10,
std::bind(&TakeoffManager::state_cb,this,std::placeholders::_1));

pose_sub_=create_subscription<geometry_msgs::msg::PoseStamped>(
"/mavros/local_position/pose",
rclcpp::SensorDataQoS(),
std::bind(&TakeoffManager::pose_cb,this,std::placeholders::_1));

object_sub_=create_subscription<
geometry_msgs::msg::PointStamped>(
"/yolov11/object_3d",10,
std::bind(&TakeoffManager::object_cb,this,std::placeholders::_1));

setpoint_pub_=create_publisher<
geometry_msgs::msg::PoseStamped>(
"/mavros/setpoint_position/local",
rclcpp::SensorDataQoS());

locked_object_pub_=create_publisher<
geometry_msgs::msg::PointStamped>(
"/cm_setpoint/locked_object_world",10);

takeoff_client_=create_client<mavros_msgs::srv::CommandTOL>(
"/mavros/cmd/takeoff");

land_client_=create_client<mavros_msgs::srv::CommandTOL>(
"/mavros/cmd/land");

takeoff_client_->wait_for_service();
land_client_->wait_for_service();

timer_=create_wall_timer(
100ms,
std::bind(&TakeoffManager::run,this));

RCLCPP_INFO(get_logger(),"CONTROL MANAGER STARTED");
}

private:

double orbit_radius_=3.0;
int orbit_points_=16;
int orbit_laps_=1;
double reach_tol_=0.30;

bool wp_reached_=false;

enum Phase{
INIT,
WAIT_ARM,
WAIT_EKF,
TAKEOFF,
HOVER1,
LOCK_OBJECT,
GENERATE_ORBIT,
ORBIT,
HOVER2,
LAND,
DONE};

Phase phase_=INIT;

mavros_msgs::msg::State state_;
geometry_msgs::msg::PoseStamped pose_;
geometry_msgs::msg::PoseStamped target_pose_;
geometry_msgs::msg::PointStamped object_cam_;

bool pose_received_=false;
bool object_received_=false;
bool takeoff_sent_=false;
bool land_sent_=false;

geometry_msgs::msg::Point object_world_;

std::vector<geometry_msgs::msg::PoseStamped> orbit_wp_;

int wp_index_=0;
int orbit_start_index_=0;   // ⭐ START WAYPOINT
int lap_count_=0;

rclcpp::Time timer_state_;

rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr object_sub_;

rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr setpoint_pub_;
rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr locked_object_pub_;

rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedPtr takeoff_client_;
rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedPtr land_client_;

rclcpp::TimerBase::SharedPtr timer_;

void state_cb(const mavros_msgs::msg::State::SharedPtr msg)
{ state_=*msg; }

void pose_cb(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
pose_=*msg;
pose_received_=true;
}

void object_cb(const geometry_msgs::msg::PointStamped::SharedPtr msg)
{
object_cam_=*msg;
object_received_=true;
}

double getYaw()
{
tf2::Quaternion q(
pose_.pose.orientation.x,
pose_.pose.orientation.y,
pose_.pose.orientation.z,
pose_.pose.orientation.w);

tf2::Matrix3x3 m(q);
double r,p,y;
m.getRPY(r,p,y);
return y;
}

void run()
{

if(!pose_received_)
{
RCLCPP_INFO_THROTTLE(get_logger(),*get_clock(),2000,
"WAITING FOR POSE...");
return;
}

if(phase_>=HOVER1 && phase_<DONE)
{
target_pose_.header.stamp=now();
setpoint_pub_->publish(target_pose_);
}

switch(phase_)
{

case INIT:
RCLCPP_INFO(get_logger(),"STATE: WAIT_ARM");
phase_=WAIT_ARM;
break;

case WAIT_ARM:
if(state_.armed)
{
RCLCPP_INFO(get_logger(),"ARM DETECTED");
timer_state_=now();
phase_=WAIT_EKF;
}
break;

case WAIT_EKF:
if((now()-timer_state_).seconds()>2.0)
{
RCLCPP_INFO(get_logger(),"STATE: TAKEOFF");
phase_=TAKEOFF;
}
break;

case TAKEOFF:
{
if(!takeoff_sent_)
{
auto req=std::make_shared<mavros_msgs::srv::CommandTOL::Request>();
req->altitude=2.0;
takeoff_client_->async_send_request(req);
takeoff_sent_=true;
}

if(pose_.pose.position.z>1.9)
{
RCLCPP_INFO(get_logger(),"ALTITUDE REACHED");
target_pose_=pose_;
timer_state_=now();
phase_=HOVER1;
}
break;
}

case HOVER1:
if((now()-timer_state_).seconds()>3.0)
{
RCLCPP_INFO(get_logger(),"STATE: LOCK_OBJECT");
phase_=LOCK_OBJECT;
}
break;

case LOCK_OBJECT:
{

if(!object_received_)
{
RCLCPP_INFO_THROTTLE(get_logger(),*get_clock(),2000,
"WAITING OBJECT...");
return;
}

double yaw=getYaw();

double xc=object_cam_.point.x;
double yc=object_cam_.point.y;

object_world_.x=
pose_.pose.position.x+
xc*cos(yaw)-yc*sin(yaw);

object_world_.y=
pose_.pose.position.y+
xc*sin(yaw)+yc*cos(yaw);

object_world_.z=
pose_.pose.position.z;

geometry_msgs::msg::PointStamped locked_msg;

locked_msg.header.stamp=now();
locked_msg.header.frame_id="map";
locked_msg.point=object_world_;

locked_object_pub_->publish(locked_msg);

RCLCPP_INFO(get_logger(),
"OBJECT LOCKED WORLD → X %.2f Y %.2f Z %.2f",
object_world_.x,
object_world_.y,
object_world_.z);

phase_=GENERATE_ORBIT;

break;
}

case GENERATE_ORBIT:
{
RCLCPP_INFO(get_logger(),"GENERATING ORBIT");

orbit_wp_.clear();

for(int i=0;i<orbit_points_;i++)
{
double ang=2*M_PI*i/orbit_points_;

geometry_msgs::msg::PoseStamped wp;

wp.pose.position.x=
object_world_.x+
orbit_radius_*cos(ang);

wp.pose.position.y=
object_world_.y+
orbit_radius_*sin(ang);

wp.pose.position.z=
object_world_.z;

orbit_wp_.push_back(wp);
}

double best=1e9;

for(size_t i=0;i<orbit_wp_.size();i++)
{
double d=hypot(
pose_.pose.position.x-
orbit_wp_[i].pose.position.x,
pose_.pose.position.y-
orbit_wp_[i].pose.position.y);

if(d<best)
{
best=d;
wp_index_=i;
}
}

orbit_start_index_=wp_index_;   // ⭐ SIMPAN START
lap_count_=0;
wp_reached_=false;

phase_=ORBIT;

break;
}

case ORBIT:
{
auto &wp=orbit_wp_[wp_index_];

double dx=
object_world_.x-
wp.pose.position.x;

double dy=
object_world_.y-
wp.pose.position.y;

double yaw=atan2(dy,dx);

tf2::Quaternion q;
q.setRPY(0,0,yaw);

target_pose_=wp;
target_pose_.pose.orientation=tf2::toMsg(q);

double dist=hypot(
pose_.pose.position.x-
wp.pose.position.x,
pose_.pose.position.y-
wp.pose.position.y);

RCLCPP_INFO_THROTTLE(
get_logger(),*get_clock(),2000,
"ORBIT WP %d | DIST %.2f",
wp_index_,dist);

if(dist < reach_tol_ && !wp_reached_)
{
wp_reached_=true;

wp_index_++;

if(wp_index_>=orbit_points_)
wp_index_=0;

if(wp_index_==orbit_start_index_)
{
lap_count_++;

RCLCPP_INFO(get_logger(),
"LAP %d COMPLETE",
lap_count_);

if(lap_count_>=orbit_laps_)
{
timer_state_=now();
phase_=HOVER2;
}
}
}

if(dist > reach_tol_)
{
wp_reached_=false;
}

break;
}

case HOVER2:
if((now()-timer_state_).seconds()>3.0)
{
RCLCPP_INFO(get_logger(),"STATE: LAND");
phase_=LAND;
}
break;

case LAND:
{
if(!land_sent_)
{
auto req=
std::make_shared<
mavros_msgs::srv::CommandTOL::Request>();

req->altitude=0.0;
land_client_->async_send_request(req);

land_sent_=true;
phase_=DONE;
}
break;
}

case DONE:
RCLCPP_INFO_THROTTLE(
get_logger(),
*get_clock(),
3000,
"MISSION COMPLETE");
break;

}

}

};

int main(int argc,char**argv)
{
rclcpp::init(argc,argv);

auto node=std::make_shared<TakeoffManager>();

rclcpp::executors::MultiThreadedExecutor executor;

executor.add_node(node);

executor.spin();

rclcpp::shutdown();
}
