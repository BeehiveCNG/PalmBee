#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/srv/command_tol.hpp>

using namespace std::chrono_literals;

class TakeoffManager : public rclcpp::Node
{
public:
    TakeoffManager() : Node("cm_setpoint")
    {
        state_sub_ =
            create_subscription<mavros_msgs::msg::State>(
                "/mavros/state",10,
                std::bind(&TakeoffManager::state_cb,this,std::placeholders::_1));

        pose_sub_ =
            create_subscription<geometry_msgs::msg::PoseStamped>(
                "/mavros/local_position/pose",
                rclcpp::SensorDataQoS(),
                std::bind(&TakeoffManager::pose_cb,this,std::placeholders::_1));

        setpoint_pub_ =
            create_publisher<geometry_msgs::msg::PoseStamped>(
                "/mavros/setpoint_position/local",
                rclcpp::SensorDataQoS());

        takeoff_client_ =
            create_client<mavros_msgs::srv::CommandTOL>(
                "/mavros/cmd/takeoff");

        land_client_ =
            create_client<mavros_msgs::srv::CommandTOL>(
                "/mavros/cmd/land");

        RCLCPP_INFO(get_logger(),"Waiting MAVROS services...");

        takeoff_client_->wait_for_service();
        land_client_->wait_for_service();

        timer_ =
            create_wall_timer(
                100ms,
                std::bind(&TakeoffManager::run,this));
    }

private:

    enum Phase
    {
        INIT,
        WAIT_ARM,
        WAIT_EKF_SETTLE,
        TAKEOFF,
        HOVER,
        LAND,
        DONE
    };

    Phase phase_ = INIT;

    mavros_msgs::msg::State state_;
    geometry_msgs::msg::PoseStamped pose_;
    geometry_msgs::msg::PoseStamped hold_pose_;

    bool pose_received_ = false;
    bool takeoff_sent_ = false;
    bool land_sent_ = false;

    rclcpp::Time arm_time_;
    rclcpp::Time hover_start_;

    //----------------------------------
    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr setpoint_pub_;

    rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedPtr takeoff_client_;
    rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedPtr land_client_;

    rclcpp::TimerBase::SharedPtr timer_;

    //----------------------------------
    template<typename FutureT>
    void wait_future(FutureT &future)
    {
        while(rclcpp::ok())
        {
            if(future.wait_for(std::chrono::milliseconds(10))
                == std::future_status::ready)
                break;
        }
    }

    //----------------------------------
    void state_cb(const mavros_msgs::msg::State::SharedPtr msg)
    { state_ = *msg; }

    void pose_cb(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        pose_ = *msg;
        pose_received_ = true;
    }

    //----------------------------------
    void run()
    {
        if(!pose_received_)
            return;

        // publish HOLD only BEFORE takeoff
        if(phase_ == INIT ||
           phase_ == WAIT_ARM ||
           phase_ == WAIT_EKF_SETTLE)
        {
            setpoint_pub_->publish(hold_pose_);
        }

        switch(phase_)
        {

        //---------------- INIT
        case INIT:
            hold_pose_ = pose_;
            RCLCPP_INFO(get_logger(),"WAIT ARM FROM MP");
            phase_ = WAIT_ARM;
            break;

        //---------------- WAIT ARM
        case WAIT_ARM:
            if(state_.armed)
            {
                RCLCPP_INFO(get_logger(),"ARM DETECTED");
                arm_time_ = now();
                phase_ = WAIT_EKF_SETTLE;
            }
            break;

        //---------------- EKF SETTLE
        case WAIT_EKF_SETTLE:
        {
            if((now()-arm_time_).seconds() > 2.0)
            {
                RCLCPP_INFO(get_logger(),
                            "EKF READY -> TAKEOFF");
                phase_ = TAKEOFF;
            }
            break;
        }

        //---------------- TAKEOFF
        case TAKEOFF:
        {
            if(!takeoff_sent_)
            {
                auto req =
                    std::make_shared<
                        mavros_msgs::srv::CommandTOL::Request>();

                req->altitude  = 1.0;
                req->latitude  = 0.0;
                req->longitude = 0.0;
                req->min_pitch = 0.0;
                req->yaw       = 0.0;

                RCLCPP_INFO(get_logger(),"Sending TAKEOFF...");

                auto future =
                    takeoff_client_->async_send_request(req);

                wait_future(future);

                auto res = future.get();

                RCLCPP_INFO(
                    get_logger(),
                    "TAKEOFF RESPONSE success=%d",
                    res->success);

                takeoff_sent_ = true;
                hover_start_ = now();
                phase_ = HOVER;
            }
            break;
        }

        //---------------- HOVER
        case HOVER:
        {
            if((now()-hover_start_).seconds()>3.0)
            {
                RCLCPP_INFO(get_logger(),
                            "HOVER DONE -> LAND");
                phase_ = LAND;
            }
            break;
        }

        //---------------- LAND
        case LAND:
        {
            if(!land_sent_)
            {
                auto req =
                    std::make_shared<
                        mavros_msgs::srv::CommandTOL::Request>();

                req->altitude = 0.0;

                RCLCPP_INFO(get_logger(),"Sending LAND...");

                auto future =
                    land_client_->async_send_request(req);

                wait_future(future);

                future.get();

                land_sent_ = true;
                phase_ = DONE;
            }
            break;
        }

        case DONE:
            break;
        }
    }
};


// ✅ MULTI THREAD REQUIRED FOR SYNC SERVICE
int main(int argc,char **argv)
{
    rclcpp::init(argc,argv);

    auto node =
        std::make_shared<TakeoffManager>();

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}

