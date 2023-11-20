#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

//#include "udp_publisher/msg/to_bort.hpp"

class Control : public rclcpp::Node {
public:
    static constexpr int TIMER_RATE_MSEC = 100;

    Control() 
    : Node("control")
    , publisher_(this->create_publisher<udp_publisher::msg::ToBort>("planner_message", 1))
    {
        send_timer_ = this->create_wall_timer(std::chrono::milliseconds(TIMER_RATE_MSEC),
            std::bind(&Control::timer_callback, this));
    }

    void timer_callback() {
        auto message = udp_publisher::msg::ToBort();
        message.yaw_joy = 0;
        message.pitch_joy= 0;
        message.roll_joy = 0;
        message.march_joy = 0;
        message.depth_joy= 0;
        message.lag_joy= 0;
        message.cs_mode = 0;
        message.beacon_x[3] = 0;
        message.beacon_y[3] = 0;
        message.yaw_closed_real = 0;
        message.pitch_closed_real = 0;
        message.roll_closed_real = 0;
        message.march_closed_real = 0;
        message.depth_closed_real = 0;
        message.lag_closed_real = 0;
        message.mode_auv_selection = 0;
        message.power_mode = 0;
        message.init_calibration = 0;
        message.save_calibration = 0;
        message.checksum_to_bort = 0;

        publisher_->publish(message);
    }

private:
    rclcpp::TimerBase::SharedPtr send_timer_;
    rclcpp::Publisher<udp_publisher::msg::ToBort>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Control>());
    rclcpp::shutdown();
    return 0;
}

