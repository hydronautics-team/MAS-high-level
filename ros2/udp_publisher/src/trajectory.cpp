#include <chrono>
#include <memory>
#include <string>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "udp_publisher/protocols.h"

#include "udp_publisher/msg/to_bort.hpp"
#include "udp_publisher/msg/from_bort.hpp"

class Trajectory : public rclcpp::Node {
public:
    static constexpr int TIMER_RATE_MSEC = 10;
    
    udp_publisher::msg::FromBort msg_from_bort_for_planner;
    udp_publisher::msg::ToBort msg_from_pult_for_planner;
    // float x_goal = 100;
    // float y_goal = 100;
    static const int NUMBER_OF_POINT = 3;
    float goal[NUMBER_OF_POINT][2] = {{100, 300}, 
                                      {150, 600}, 
                                      {100, 900}}; 
    int number_of_point = 0;

    Trajectory() 
    : Node("control")
    , publisher_to_bort(this->create_publisher<udp_publisher::msg::ToBort>("planner_message", 1))
    , publisher_to_pult(this->create_publisher<udp_publisher::msg::FromBort>("planner_feedback", 1))
    , subscription_from_config(this->create_subscription<std_msgs::msg::String>("name_config_file", 1, std::bind(&Trajectory::config_file_callback, this, std::placeholders::_1)))
    , subscription_from_bort(this->create_subscription<udp_publisher::msg::FromBort>("udp_message", 1, std::bind(&Trajectory::udp_message_callback, this, std::placeholders::_1)))
    , subscription_from_pult(this->create_subscription<udp_publisher::msg::ToBort>("udp_message_pult", 1, std::bind(&Trajectory::udp_message_pult_callback, this, std::placeholders::_1)))
    {}

void config_file_callback(std_msgs::msg::String const &conf_string) { 
    conf_str = conf_string.data;
    RCLCPP_INFO_STREAM(this->get_logger(), conf_str);
    //сейчас заданное ручное управление, потом - следование
    if (conf_str == "go_along_trajectory") {
        timer_trajectory = this->create_wall_timer(std::chrono::milliseconds(TIMER_RATE_MSEC),
        std::bind(&Trajectory::timer_trajectory_callback, this));    
    } else {//отправляем на пульт ошибку, управление на борт не шлем!!!!!
        // auto message = udp_publisher::msg::ToBort();  
        // message.yaw_joy = msg_from_pult_to_planner.controlData.yaw;
        // message.pitch_joy = msg_from_pult_to_planner.controlData.pitch;
        // message.roll_joy = msg_from_pult_to_planner.controlData.roll;
        // message.march_joy = msg_from_pult_to_planner.controlData.march;
        // message.depth_joy = msg_from_pult_to_planner.controlData.depth;
        // message.lag_joy = msg_from_pult_to_planner.controlData.lag;
        // message.cs_mode = uint8_t(msg_from_pult_to_planner.cSMode);
        // и т. д. 
        
    }
}

void timer_trajectory_callback() {
    auto message_to_bort = udp_publisher::msg::ToBort();
    auto message_to_pult = udp_publisher::msg::FromBort(); 

    if (msg_from_pult_for_planner.mission_command == 4) { //миссия завершена
        //остановить таймер и выйти из метода
        RCLCPP_INFO_STREAM(this->get_logger(), "I know that you want complete mission trajectory");
        timer_trajectory->cancel();
        return;
    }
    RCLCPP_INFO_STREAM(this->get_logger(), "A_am_in_mission_trajectory");

    float x_v_SK_sv_x_y_goal = msg_from_bort_for_planner.location_x - goal[number_of_point][0]; //координаты аппаратов, в случае СК с началом координат в точке-цели
    float y_v_SK_sv_x_y_goal = msg_from_bort_for_planner.location_y - goal[number_of_point][1];

    RCLCPP_INFO_STREAM(this->get_logger(), "^^^^^^^^^^^^^^^^^^^^^"); 
    RCLCPP_INFO_STREAM(this->get_logger(), "coordinate_in_SK_goal_point"); 
    RCLCPP_INFO_STREAM(this->get_logger(), x_v_SK_sv_x_y_goal); 
    RCLCPP_INFO_STREAM(this->get_logger(), y_v_SK_sv_x_y_goal); 
    RCLCPP_INFO_STREAM(this->get_logger(), "^^^^^^^^^^^^^^^^^^^^^"); 

    RCLCPP_INFO_STREAM(this->get_logger(), "^^^^^^^^^^^^^^^^^^^^^"); 
    RCLCPP_INFO_STREAM(this->get_logger(), "radius mezdy point"); 
    RCLCPP_INFO_STREAM(this->get_logger(), sqrt(pow((msg_from_bort_for_planner.location_x - goal[number_of_point][0]), 2) + pow((msg_from_bort_for_planner.location_y- goal[number_of_point][1]), 2)));
    RCLCPP_INFO_STREAM(this->get_logger(), "^^^^^^^^^^^^^^^^^^^^^"); 

    if (sqrt(pow((msg_from_bort_for_planner.location_x - goal[number_of_point][0]), 2) + pow((msg_from_bort_for_planner.location_y- goal[number_of_point][1]), 2)) < 30) {  
        if (number_of_point >= NUMBER_OF_POINT-1)  {
            RCLCPP_INFO_STREAM(this->get_logger(), "You are in two meters radius");
            message_to_bort = msg_from_pult_for_planner;
            message_to_bort.yaw_joy = 0;
            message_to_bort.pitch_joy= 0;
            message_to_bort.roll_joy = 0;
            message_to_bort.march_joy = 0;
            message_to_bort.depth_joy= 0;
            message_to_bort.lag_joy= 0;
            message_to_bort.cs_mode = 2;
            message_to_bort.yaw_closed_real = 0;
            message_to_bort.pitch_closed_real = 0;
            message_to_bort.roll_closed_real = 0;
            message_to_bort.march_closed_real = 0;
            message_to_bort.depth_closed_real = 0;
            message_to_bort.lag_closed_real = 0;
            message_to_bort.power_mode = 2;
            message_to_bort.init_calibration = 0;
            message_to_bort.save_calibration = 0;
            message_to_bort.id_mission_auv = 1;
            message_to_bort.checksum_to_bort = 0;

            message_to_pult = msg_from_bort_for_planner;          
            message_to_pult.id_mission = 2;
            message_to_pult.mission_status = 4;

            timer_trajectory->cancel();


            } else {
                number_of_point++;  
                RCLCPP_INFO_STREAM(this->get_logger(), "moved to next point");
                message_to_bort = msg_from_pult_for_planner;
                message_to_bort.yaw_joy = 0;
                message_to_bort.pitch_joy= 0;
                message_to_bort.roll_joy = 0;
                message_to_bort.march_joy = 0;
                message_to_bort.depth_joy= 0;
                message_to_bort.lag_joy= 0;
                message_to_bort.cs_mode = 2;
                message_to_bort.yaw_closed_real = 0;
                message_to_bort.pitch_closed_real = 0;
                message_to_bort.roll_closed_real = 0;
                message_to_bort.march_closed_real = 0;
                message_to_bort.depth_closed_real = 0;
                message_to_bort.lag_closed_real = 0;
                //message_to_bort.power_mode = 2;
                message_to_bort.init_calibration = 0;
                message_to_bort.save_calibration = 0;
                message_to_bort.id_mission_auv = 1;
                message_to_bort.checksum_to_bort = 0;

                message_to_pult = msg_from_bort_for_planner;          
                message_to_pult.id_mission = 2;
                message_to_pult.mission_status = 2;
            }
        } else {
            RCLCPP_INFO_STREAM(this->get_logger(), "^^^^^^^^^^^^^^^^^^^^^"); 
            RCLCPP_INFO_STREAM(this->get_logger(), "moved to point:");  
            RCLCPP_INFO_STREAM(this->get_logger(), number_of_point + 1);  
            RCLCPP_INFO_STREAM(this->get_logger(), "^^^^^^^^^^^^^^^^^^^^^");  
            message_to_bort = msg_from_pult_for_planner;
            if (x_v_SK_sv_x_y_goal < 0) {  //левая полуплоскость относительно целевой точки
                if (y_v_SK_sv_x_y_goal <= 0) {   // 3 четверть
                    message_to_bort.yaw_joy = atan(y_v_SK_sv_x_y_goal/x_v_SK_sv_x_y_goal)*(180/M_PI); // расчет угла курса и его перевод в градусы
                }
                if ((y_v_SK_sv_x_y_goal > 0)) {
                    message_to_bort.yaw_joy = atan(y_v_SK_sv_x_y_goal/x_v_SK_sv_x_y_goal)*(180/M_PI); // 2 четверть
                }
            }
            if (x_v_SK_sv_x_y_goal >= 0) {
                if (y_v_SK_sv_x_y_goal > 0) { // правая полуплоскость относительно целевой точки
                    message_to_bort.yaw_joy = -180 + atan(y_v_SK_sv_x_y_goal/x_v_SK_sv_x_y_goal)*(180/M_PI); // 1 четверть
                }
                if (y_v_SK_sv_x_y_goal <= 0) {
                    message_to_bort.yaw_joy = 180 + atan(y_v_SK_sv_x_y_goal/x_v_SK_sv_x_y_goal)*(180/M_PI); // 4 четверть
                }
            }
            
            message_to_bort.pitch_joy= 0;
            message_to_bort.roll_joy = 0;
            message_to_bort.march_joy = 10;
            message_to_bort.depth_joy= 0;
            message_to_bort.lag_joy= 0;
            message_to_bort.cs_mode = 2;
            message_to_bort.yaw_closed_real = 1;
            message_to_bort.pitch_closed_real = 0;
            message_to_bort.roll_closed_real = 0;
            message_to_bort.march_closed_real = 0;
            message_to_bort.depth_closed_real = 0;
            message_to_bort.lag_closed_real = 0;
            message_to_bort.id_mission_auv = 1;
            message_to_bort.checksum_to_bort = 0;

            message_to_pult= msg_from_bort_for_planner;  
            message_to_pult.id_mission = 2;
            message_to_pult.mission_status = 2;
        }

        publisher_to_bort->publish(message_to_bort);
        publisher_to_pult->publish(message_to_pult);
    
}

void udp_message_pult_callback(udp_publisher::msg::ToBort const &msg_from_pult) {
    msg_from_pult_for_planner.yaw_joy = msg_from_pult.yaw_joy;
    msg_from_pult_for_planner.pitch_joy = msg_from_pult.pitch_joy;
    msg_from_pult_for_planner.roll_joy = msg_from_pult.roll_joy;
    msg_from_pult_for_planner.march_joy = msg_from_pult.march_joy;
    msg_from_pult_for_planner.depth_joy = msg_from_pult.depth_joy;
    msg_from_pult_for_planner.lag_joy = msg_from_pult.lag_joy;

    msg_from_pult_for_planner.cs_mode = msg_from_pult.cs_mode;

    msg_from_pult_for_planner.beacon_x[3] = msg_from_pult.beacon_x[3];
    msg_from_pult_for_planner.beacon_y[3] = msg_from_pult.beacon_y[3];

    msg_from_pult_for_planner.yaw_closed_real = msg_from_pult.yaw_closed_real;
    msg_from_pult_for_planner.pitch_closed_real = msg_from_pult.pitch_closed_real;
    msg_from_pult_for_planner.roll_closed_real = msg_from_pult.roll_closed_real;
    msg_from_pult_for_planner.march_closed_real = msg_from_pult.march_closed_real;
    msg_from_pult_for_planner.depth_closed_real = msg_from_pult.depth_closed_real;
    msg_from_pult_for_planner.lag_closed_real = msg_from_pult.lag_closed_real;

    msg_from_pult_for_planner.mode_auv_selection = msg_from_pult.mode_auv_selection;

    msg_from_pult_for_planner.power_mode = msg_from_pult.power_mode;

    msg_from_pult_for_planner.init_calibration = msg_from_pult.init_calibration;
    msg_from_pult_for_planner.save_calibration = msg_from_pult.save_calibration;

    msg_from_pult_for_planner.id_mission_auv = msg_from_pult.id_mission_auv;
    msg_from_pult_for_planner.mission_command = msg_from_pult.mission_command;

    msg_from_pult_for_planner.checksum_to_bort = msg_from_pult.checksum_to_bort;
    }

    void udp_message_callback(udp_publisher::msg::FromBort const &msg_from_bort) {

    msg_from_bort_for_planner.sender_id = msg_from_bort.sender_id;
    msg_from_bort_for_planner.receiver_id= msg_from_bort.receiver_id;
    msg_from_bort_for_planner.msg_size = msg_from_bort.msg_size;

    msg_from_bort_for_planner.mode_real = msg_from_bort.mode_real;

    msg_from_bort_for_planner.yaw_contour = msg_from_bort.yaw_contour;
    msg_from_bort_for_planner.pitch_contour = msg_from_bort.pitch_contour;
    msg_from_bort_for_planner.roll_contour = msg_from_bort.roll_contour;
    msg_from_bort_for_planner.march_contour = msg_from_bort.march_contour;
    msg_from_bort_for_planner.depth_contour = msg_from_bort.depth_contour;
    msg_from_bort_for_planner.lag_contour = msg_from_bort.lag_contour;

    msg_from_bort_for_planner.mode_auv_real = msg_from_bort.mode_auv_real;

    msg_from_bort_for_planner.yaw = msg_from_bort.yaw;
    msg_from_bort_for_planner.pitch = msg_from_bort.pitch;
    msg_from_bort_for_planner.roll = msg_from_bort.roll;
    msg_from_bort_for_planner.march = msg_from_bort.march;
    msg_from_bort_for_planner.depth = msg_from_bort.depth;
    msg_from_bort_for_planner.lag = msg_from_bort.lag;

    msg_from_bort_for_planner.vma1 = msg_from_bort.vma1;
    msg_from_bort_for_planner.vma2 = msg_from_bort.vma2;
    msg_from_bort_for_planner.vma3 = msg_from_bort.vma3;
    msg_from_bort_for_planner.vma4 = msg_from_bort.vma4;
    msg_from_bort_for_planner.vma5 = msg_from_bort.vma5;
    msg_from_bort_for_planner.vma6 = msg_from_bort.vma6;

    msg_from_bort_for_planner.yaw_imu = msg_from_bort.yaw_imu;
    msg_from_bort_for_planner.pitch_imu = msg_from_bort.pitch_imu;
    msg_from_bort_for_planner.roll_imu = msg_from_bort.roll_imu;
    msg_from_bort_for_planner.x_accel_imu = msg_from_bort.x_accel_imu;
    msg_from_bort_for_planner.y_accel_imu = msg_from_bort.y_accel_imu;
    msg_from_bort_for_planner.z_accel_imu = msg_from_bort.z_accel_imu;
    msg_from_bort_for_planner.x_rate_imu = msg_from_bort.x_rate_imu;
    msg_from_bort_for_planner.y_rate_imu = msg_from_bort.y_rate_imu;
    msg_from_bort_for_planner.z_rate_imu = msg_from_bort.z_rate_imu;
    msg_from_bort_for_planner.x_magn_imu = msg_from_bort.x_magn_imu;
    msg_from_bort_for_planner.y_magn_imu = msg_from_bort.y_magn_imu;
    msg_from_bort_for_planner.z_magn_imu= msg_from_bort.z_magn_imu;
    msg_from_bort_for_planner.quat[4] = msg_from_bort.quat[4];

    msg_from_bort_for_planner.temperature = msg_from_bort.temperature;
    msg_from_bort_for_planner.depth_sensor = msg_from_bort.depth_sensor;
    msg_from_bort_for_planner.pressure = msg_from_bort.pressure;

    msg_from_bort_for_planner.error_code_uwb = msg_from_bort.error_code_uwb;
    msg_from_bort_for_planner.connection_field_uwb = msg_from_bort.connection_field_uwb;
    msg_from_bort_for_planner.location_x = msg_from_bort.location_x;
    msg_from_bort_for_planner.location_y = msg_from_bort.location_y;
    msg_from_bort_for_planner.distance_to_beacon[4] = msg_from_bort.distance_to_beacon[4];
    msg_from_bort_for_planner.distance_to_agent[10] = msg_from_bort.distance_to_agent[10];

    msg_from_bort_for_planner.start_calibration = msg_from_bort.start_calibration;
    msg_from_bort_for_planner.end_calibration= msg_from_bort.end_calibration;

    msg_from_bort_for_planner.id_mission= msg_from_bort.id_mission;
    msg_from_bort_for_planner.mission_status = msg_from_bort.mission_status;

    msg_from_bort_for_planner.checksum = 0;
    } 

private:
    rclcpp::TimerBase::SharedPtr timer_trajectory;
    rclcpp::Publisher<udp_publisher::msg::ToBort>::SharedPtr publisher_to_bort;
    rclcpp::Publisher<udp_publisher::msg::FromBort>::SharedPtr publisher_to_pult;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_from_config;
    rclcpp::Subscription<udp_publisher::msg::FromBort>::SharedPtr subscription_from_bort;
    rclcpp::Subscription<udp_publisher::msg::ToBort>::SharedPtr subscription_from_pult;
    std::string conf_str;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Trajectory>());
    rclcpp::shutdown();
    return 0;
}