#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "udp_publisher/msg/to_bort.hpp"
#include "udp_publisher/msg/from_bort.hpp"

class Control : public rclcpp::Node {
public:
    static constexpr int TIMER_RATE_MSEC = 100;
    int flag_start_yaw = 1;
    int DURATION = 300;
    int count_duration = 0;

    Control() 
    : Node("control")
    , publisher_from_pult(this->create_publisher<udp_publisher::msg::ToBort>("planner_message", 1))
    , publisher_to_pult(this->create_publisher<udp_publisher::msg::FromBort>("planner_feedback", 1))
    , subscription_from_config(this->create_subscription<std_msgs::msg::String>("name_config_file", 1, std::bind(&Control::config_file_callback, this, std::placeholders::_1)))
    
    {
        send_timer_ = this->create_wall_timer(std::chrono::milliseconds(TIMER_RATE_MSEC),
            std::bind(&Control::timer_callback, this));
    }

private:
    void config_file_callback(std_msgs::msg::String const &conf_string) { 
        // std::string conf_str = (char*)&conf_string;
        // RCLCPP_INFO_STREAM(this->get_logger(), conf_str);
        //if (conf_str == "Go_with_yaw_and_peleng") {
            auto message = udp_publisher::msg::ToBort();
            
            if (count_duration < DURATION) {                
                message.yaw_joy = -7;    
                message.pitch_joy= 0;
                message.roll_joy = 0;
                message.march_joy = 3;
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
                message.power_mode = 2;
                message.init_calibration = 0;
                message.save_calibration = 0;          
                message.id_mission_auv = 2;
                message.mission_command = 1;
                message.checksum_to_bort = 3;

                flag_start_yaw = 0;
                count_duration++;
            } 
            if (count_duration >= DURATION) {
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
                message.power_mode = 2;
                message.init_calibration = 0;
                message.save_calibration = 0;
                message.id_mission_auv = 2;
                message.mission_command = 4;
                message.checksum_to_bort = 0;
            }
            RCLCPP_INFO_STREAM(this->get_logger(), "cnt " << count_duration  );
            publisher_from_pult->publish(message);
        //}
    }    

    void timer_callback() {
        
    }

private:
    rclcpp::TimerBase::SharedPtr send_timer_;
    rclcpp::Publisher<udp_publisher::msg::ToBort>::SharedPtr publisher_from_pult;
    rclcpp::Publisher<udp_publisher::msg::FromBort>::SharedPtr publisher_to_pult;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_from_config;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Control>());
    rclcpp::shutdown();
    return 0;
}

