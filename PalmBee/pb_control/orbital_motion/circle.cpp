#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <cmath>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>

using namespace std::chrono_literals;
using namespace px4_msgs::msg;
using geometry_msgs::msg::PointStamped;

class OffboardControl : public rclcpp::Node {
public:
    OffboardControl() : Node("offboard_control"), has_received_target_(false), angle_(0.0), is_paused_(false) {
        trajectory_setpoint_publisher_ = this->create_publisher<TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
        circle_trajectory_publisher_ = this->create_publisher<VehicleLocalPosition>("/circle_trajectory", 10);

        target_position_subscriber_ = this->create_subscription<PointStamped>(
            "/palmbee/tree/target_position", 10,
            std::bind(&OffboardControl::target_position_callback, this, std::placeholders::_1));

        timer_ = this->create_wall_timer(100ms, std::bind(&OffboardControl::publish_trajectory_setpoint, this));
        
        std::thread(&OffboardControl::read_keyboard_input, this).detach();
        set_terminal_mode();
    }

    ~OffboardControl() { reset_terminal_mode(); }

private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
    rclcpp::Publisher<VehicleLocalPosition>::SharedPtr circle_trajectory_publisher_;
    rclcpp::Subscription<PointStamped>::SharedPtr target_position_subscriber_;
    
    double angle_;
    bool is_paused_;
    bool has_received_target_;
    double x_center_;
    double y_center_;
    struct termios oldt_;

    void publish_trajectory_setpoint();
    void target_position_callback(const PointStamped::SharedPtr msg);
    void read_keyboard_input();
    void set_terminal_mode();
    void reset_terminal_mode();
};

void OffboardControl::target_position_callback(const PointStamped::SharedPtr msg) {
    x_center_ = msg->point.x;
    y_center_ = msg->point.y;
    has_received_target_ = true;
    RCLCPP_INFO(this->get_logger(), "Updated target position: X: %.2f, Y: %.2f", x_center_, y_center_);
}

void OffboardControl::publish_trajectory_setpoint() {
    if (!has_received_target_ || is_paused_) return;

    double radius = 1.5;
    double angular_speed = 0.02;
    double x = x_center_ + radius * cos(angle_);
    double y = y_center_ + radius * sin(angle_);
    double yaw = std::atan2(-(y - y_center_), -(x - x_center_));

    angle_ = std::fmod(angle_ + angular_speed, 2 * M_PI);

    TrajectorySetpoint msg{};
    msg.position = {static_cast<float>(x), static_cast<float>(y), -5.0f};
    msg.yaw = static_cast<float>(yaw);
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    trajectory_setpoint_publisher_->publish(msg);

    VehicleLocalPosition circle_point;
    circle_point.x = static_cast<float>(x);
    circle_point.y = static_cast<float>(y);
    circle_point.z = 0.0f;
    circle_trajectory_publisher_->publish(circle_point);
}

void OffboardControl::read_keyboard_input() {
    while (rclcpp::ok()) {
        char ch = getchar();
        if (ch == 'p' || ch == 'P') {
            is_paused_ = !is_paused_;
            RCLCPP_INFO(this->get_logger(), "Pause is %s.", is_paused_ ? "ON" : "OFF");
        }
    }
}

void OffboardControl::set_terminal_mode() {
    struct termios newt;
    tcgetattr(STDIN_FILENO, &oldt_);
    newt = oldt_;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
}

void OffboardControl::reset_terminal_mode() {
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt_);
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OffboardControl>());
    rclcpp::shutdown();
    return 0;
}