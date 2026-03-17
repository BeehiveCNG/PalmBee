#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <mavros_msgs/srv/set_mode.hpp>
#include <mavros_msgs/srv/command_bool.hpp>

using namespace std::chrono_literals;

enum class FlightState
{
    WAIT_EKF,
    SET_GUIDED,
    ARM,
    TAKEOFF,
    HOVER1,
    FORWARD,
    HOVER2
};

class ControlManager : public rclcpp::Node
{
public:
    ControlManager() : Node("control_manager")
    {
        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            "/mavros/local_position/odom",
            rclcpp::SensorDataQoS(),
            std::bind(&ControlManager::odom_cb, this, std::placeholders::_1));

        vel_pub_ = create_publisher<geometry_msgs::msg::Twist>(
            "/mavros/setpoint_velocity/cmd_vel_unstamped", 10);

        mode_client_ = create_client<mavros_msgs::srv::SetMode>("/mavros/set_mode");
        arm_client_ = create_client<mavros_msgs::srv::CommandBool>("/mavros/cmd/arming");

        timer_ = create_wall_timer(50ms, std::bind(&ControlManager::loop, this));
    }

private:
    FlightState state_ = FlightState::WAIT_EKF;

    double current_z_ = 0.0;
    double start_x_ = 0.0;
    double current_x_ = 0.0;
    bool got_start_x_ = false;
    int hover_count_ = 0;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
    rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr mode_client_;
    rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arm_client_;
    rclcpp::TimerBase::SharedPtr timer_;

    void odom_cb(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        current_z_ = msg->pose.pose.position.z;
        current_x_ = msg->pose.pose.position.x;

        if (!got_start_x_) {
            start_x_ = current_x_;
            got_start_x_ = true;
        }
    }

    void set_mode(const std::string &mode)
    {
        if (!mode_client_->service_is_ready()) return;
        auto req = std::make_shared<mavros_msgs::srv::SetMode::Request>();
        req->custom_mode = mode;
        mode_client_->async_send_request(req);
    }

    void arm()
    {
        if (!arm_client_->service_is_ready()) return;
        auto req = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
        req->value = true;
        arm_client_->async_send_request(req);
    }

    void send_velocity(double vx, double vy, double vz)
    {
        geometry_msgs::msg::Twist cmd;
        cmd.linear.x = vx;
        cmd.linear.y = vy;
        cmd.linear.z = vz;
        vel_pub_->publish(cmd);
    }

    void loop()
    {
        switch (state_)
        {
        case FlightState::WAIT_EKF:
            send_velocity(0,0,0);
            if (fabs(current_z_) < 0.05) {
                RCLCPP_INFO(get_logger(), "EKF ready");
                state_ = FlightState::SET_GUIDED;
            }
            break;

        case FlightState::SET_GUIDED:
            set_mode("GUIDED_NOGPS");
            RCLCPP_INFO(get_logger(), "GUIDED_NOGPS");
            state_ = FlightState::ARM;
            break;

        case FlightState::ARM:
            arm();
            RCLCPP_INFO(get_logger(), "ARMED");
            state_ = FlightState::TAKEOFF;
            break;

        case FlightState::TAKEOFF:
            send_velocity(0, 0, -0.6);   // go UP
            if (current_z_ > 1.4) {
                RCLCPP_INFO(get_logger(), "Takeoff reached");
                state_ = FlightState::HOVER1;
                hover_count_ = 0;
            }
            break;

        case FlightState::HOVER1:
            send_velocity(0,0,0);
            hover_count_++;
            if (hover_count_ > 60) { // 3 sec
                RCLCPP_INFO(get_logger(), "Forward");
                state_ = FlightState::FORWARD;
            }
            break;

        case FlightState::FORWARD:
            send_velocity(0.5, 0, 0);
            if (fabs(current_x_ - start_x_) > 2.0) {
                RCLCPP_INFO(get_logger(), "Reached 2m");
                state_ = FlightState::HOVER2;
            }
            break;

        case FlightState::HOVER2:
            send_velocity(0,0,0);
            break;
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ControlManager>());
    rclcpp::shutdown();
    return 0;
}

