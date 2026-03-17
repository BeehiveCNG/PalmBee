#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/srv/command_tol.hpp>

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

setpoint_pub_=create_publisher<geometry_msgs::msg::PoseStamped>(
"/mavros/setpoint_position/local",
rclcpp::SensorDataQoS());

takeoff_client_=create_client<mavros_msgs::srv::CommandTOL>(
"/mavros/cmd/takeoff");

land_client_=create_client<mavros_msgs::srv::CommandTOL>(
"/mavros/cmd/land");

takeoff_client_->wait_for_service();
land_client_->wait_for_service();

timer_=create_wall_timer(
100ms,
std::bind(&TakeoffManager::run,this));
}

private:

enum Phase{
INIT,
WAIT_ARM,
WAIT_EKF,
TAKEOFF,
HOVER1,
MOVE_FORWARD,
HOVER2,
LAND,
DONE
};

Phase phase_=INIT;

mavros_msgs::msg::State state_;
geometry_msgs::msg::PoseStamped pose_;
geometry_msgs::msg::PoseStamped hold_pose_;
geometry_msgs::msg::PoseStamped target_pose_;

bool pose_received_=false;
bool takeoff_sent_=false;
bool land_sent_=false;
bool move_init_=false;

double start_x_;

rclcpp::Time state_timer_;

rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr setpoint_pub_;

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

void run()
{
if(!pose_received_)
return;

/* publish setpoint continuously after airborne */
if(phase_>=HOVER1 && phase_<LAND)
setpoint_pub_->publish(target_pose_);

if(phase_<=WAIT_EKF)
setpoint_pub_->publish(hold_pose_);

switch(phase_)
{

case INIT:
hold_pose_=pose_;
RCLCPP_INFO(get_logger(),"WAIT ARM");
phase_=WAIT_ARM;
break;

case WAIT_ARM:
if(state_.armed)
{
RCLCPP_INFO(get_logger(),"ARM DETECTED");
state_timer_=now();
phase_=WAIT_EKF;
}
break;

case WAIT_EKF:
if((now()-state_timer_).seconds()>2.0)
{
RCLCPP_INFO(get_logger(),"TAKEOFF");
phase_=TAKEOFF;
}
break;

/* ========= TAKEOFF ========= */
case TAKEOFF:
{
if(!takeoff_sent_)
{
auto req=
std::make_shared<mavros_msgs::srv::CommandTOL::Request>();

req->altitude=1.0;
req->latitude=0.0;
req->longitude=0.0;
req->min_pitch=0.0;
req->yaw=0.0;

takeoff_client_->async_send_request(req);

takeoff_sent_=true;
}

if(pose_.pose.position.z>0.9)
{
RCLCPP_INFO(get_logger(),"ALTITUDE REACHED");

target_pose_=pose_;
state_timer_=now();
phase_=HOVER1;
}
break;
}

/* ========= HOVER ========= */
case HOVER1:
if((now()-state_timer_).seconds()>3.0)
{
RCLCPP_INFO(get_logger(),"MOVE FORWARD 5m");

start_x_=pose_.pose.position.x;
target_pose_=pose_;
target_pose_.pose.position.x+=5.0;

phase_=MOVE_FORWARD;
}
break;

/* ========= MOVE ========= */
case MOVE_FORWARD:
{
double moved=
pose_.pose.position.x-start_x_;

if(moved>4.8)
{
RCLCPP_INFO(get_logger(),"TARGET REACHED");

state_timer_=now();
phase_=HOVER2;
}
break;
}

/* ========= HOVER ========= */
case HOVER2:
if((now()-state_timer_).seconds()>3.0)
{
RCLCPP_INFO(get_logger(),"LAND");
phase_=LAND;
}
break;

/* ========= LAND ========= */
case LAND:
{
if(!land_sent_)
{
auto req=
std::make_shared<mavros_msgs::srv::CommandTOL::Request>();

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
