#include <chrono>
#include <memory>
#include <string>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "udp_publisher/protocols.h"

#include "udp_publisher/msg/to_bort.hpp"
#include "udp_publisher/msg/from_bort.hpp"

class Control : public rclcpp::Node {
public:
    static constexpr int TIMER_RATE_MSEC = 100;
    int flag_start_yaw = 1;
    int DURATION = 300;
    int count_duration = 0;
    FromBort msg_from_bort_for_planner;
    ToBort msg_from_pult_for_planner;
    float x_goal = 5;
    float y_goal = -5;
    int modelling_flag = 1;

    Control() 
    : Node("control")
    , publisher_to_bort(this->create_publisher<udp_publisher::msg::ToBort>("planner_message", 1))
    , publisher_to_pult(this->create_publisher<udp_publisher::msg::FromBort>("planner_feedback", 1))
    , subscription_from_config(this->create_subscription<std_msgs::msg::String>("name_config_file", 1, std::bind(&Control::config_file_callback, this, std::placeholders::_1)))
    , subscription_from_bort(this->create_subscription<udp_publisher::msg::FromBort>("udp_message", 1, std::bind(&Control::udp_message_callback, this, std::placeholders::_1)))
    , subscription_from_pult(this->create_subscription<udp_publisher::msg::ToBort>("udp_message_pult", 1, std::bind(&Control::udp_message_pult_callback, this, std::placeholders::_1)))

    {
        RCLCPP_INFO_STREAM(this->get_logger(), "sizeof(udp_publisher::msg::ToBort)");
        RCLCPP_INFO_STREAM(this->get_logger(), sizeof(udp_publisher::msg::ToBort));
    }

private:
    uint checksum_i(const void * data, int size){ // function for checksum (uint)
        uint c = 0;
        for (int i = 0; i < size; ++i)
            c += ((const unsigned char*)data)[i];
        return ~(c + 1);
    }
    void config_file_callback(std_msgs::msg::String const &conf_string) { 
        conf_str = conf_string.data;
        RCLCPP_INFO_STREAM(this->get_logger(), conf_str);
        //сейчас заданное ручное управление, потом - следование
        if (conf_str == "go_with_yaw_and_peleng") {
            timer_following = this->create_wall_timer(std::chrono::milliseconds(TIMER_RATE_MSEC),
            std::bind(&Control::timer_following_callback, this));    
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

        //выход в точку
        if (conf_str == "go_to_point") {
            RCLCPP_INFO_STREAM(this->get_logger(), "!!!!!!!!!!!!!!!!!!!!!!!");
            timer_exit_to_point = this->create_wall_timer(std::chrono::milliseconds(TIMER_RATE_MSEC),
                std::bind(&Control::timer_exit_to_point_callback, this));
        }
    }    

    void timer_following_callback() {
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
            publisher_to_bort->publish(message);
    }
    
    void timer_exit_to_point_callback() {

        auto message_to_bort = udp_publisher::msg::ToBort();
        auto message_to_pult = udp_publisher::msg::FromBort(); 

        if (msg_from_pult_for_planner.missionControl == mission_Control::MODE_COMPLETE) {
            //остановить таймер и выйти из метода
            RCLCPP_INFO_STREAM(this->get_logger(), "I know that you want complete mission");

            timer_exit_to_point->cancel();

            // message_to_bort.yaw_joy = 0;
            // message_to_bort.pitch_joy= 0;
            // message_to_bort.roll_joy = 0;
            // message_to_bort.march_joy = 0;
            // message_to_bort.depth_joy= 0;
            // message_to_bort.lag_joy= 0;
            // message_to_bort.cs_mode = 2;
            // message_to_bort.beacon_x[3] = msg_from_pult_for_planner.pultUWB.beacon_x[3];
            // message_to_bort.beacon_y[3] = msg_from_pult_for_planner.pultUWB.beacon_y[3];
            // message_to_bort.yaw_closed_real = 0;
            // message_to_bort.pitch_closed_real = 0;
            // message_to_bort.roll_closed_real = 0;
            // message_to_bort.march_closed_real = 0;
            // message_to_bort.depth_closed_real = 0;
            // message_to_bort.lag_closed_real = 0;
            // message_to_bort.mode_auv_selection = 0;
            // message_to_bort.power_mode = 2;
            // message_to_bort.init_calibration = 0;
            // message_to_bort.save_calibration = 0;
            // message_to_bort.id_mission_auv = 0;
            // message_to_bort.mission_command = 0;
            // message_to_bort.checksum_to_bort = 0;
            // publisher_to_bort->publish(message_to_bort);
            
            return;
        }
        RCLCPP_INFO_STREAM(this->get_logger(), "A_am_in_mission_go_to_point");
        

        float x_v_SK_sv_x_y_goal = msg_from_bort_for_planner.dataUWB.locationX - x_goal; //координаты аппаратов, в случае СК с началом координат в точке-цели
        float y_v_SK_sv_x_y_goal = msg_from_bort_for_planner.dataUWB.locationY- y_goal;
        // RCLCPP_INFO_STREAM(this->get_logger(), msg_from_bort_for_planner.dataUWB.locationX);
        // RCLCPP_INFO_STREAM(this->get_logger(), x_goal);
        // RCLCPP_INFO_STREAM(this->get_logger(), x_v_SK_sv_x_y_goal);
        // RCLCPP_INFO_STREAM(this->get_logger(), "-------------");

        // проверка расстояния до целевой точки, если < 2м, то размыкаю контура
        if (sqrt(pow((msg_from_bort_for_planner.dataUWB.locationX - x_goal), 2) + pow((msg_from_bort_for_planner.dataUWB.locationY - y_goal), 2)) < 3) {  
            RCLCPP_INFO_STREAM(this->get_logger(), "You are in two meters radius");
            message_to_bort.yaw_joy = 0;
            message_to_bort.pitch_joy= 0;
            message_to_bort.roll_joy = 0;
            message_to_bort.march_joy = 0;
            message_to_bort.depth_joy= 0;
            message_to_bort.lag_joy= 0;
            message_to_bort.cs_mode = 2;
            message_to_bort.beacon_x[3] = msg_from_pult_for_planner.pultUWB.beacon_x[3];
            message_to_bort.beacon_y[3] = msg_from_pult_for_planner.pultUWB.beacon_y[3];
            message_to_bort.yaw_closed_real = 0;
            message_to_bort.pitch_closed_real = 0;
            message_to_bort.roll_closed_real = 0;
            message_to_bort.march_closed_real = 0;
            message_to_bort.depth_closed_real = 0;
            message_to_bort.lag_closed_real = 0;
            message_to_bort.mode_auv_selection = uint8_t(msg_from_pult_for_planner.cSMode);
            message_to_bort.power_mode = 2;
            message_to_bort.init_calibration = 0;
            message_to_bort.save_calibration = 0;
            message_to_bort.id_mission_auv = 1;
            message_to_bort.mission_command = uint8_t(msg_from_pult_for_planner.missionControl);
            message_to_bort.checksum_to_bort = 0;

            message_to_pult.sender_id = msg_from_bort_for_planner.headerSwap.senderID;
            message_to_pult.receiver_id = msg_from_bort_for_planner.headerSwap.receiverID;
            message_to_pult.msg_size = msg_from_bort_for_planner.headerSwap.msgSize;

            message_to_pult.mode_real = msg_from_bort_for_planner.auvData.modeReal;

            message_to_pult.yaw_contour = msg_from_bort_for_planner.auvData.controlReal.yaw;
            message_to_pult.pitch_contour = msg_from_bort_for_planner.auvData.controlReal.pitch;
            message_to_pult.roll_contour = msg_from_bort_for_planner.auvData.controlReal.roll;
            message_to_pult.march_contour = msg_from_bort_for_planner.auvData.controlReal.march;
            message_to_pult.depth_contour = msg_from_bort_for_planner.auvData.controlReal.depth;
            message_to_pult.lag_contour = msg_from_bort_for_planner.auvData.controlReal.lag;

            message_to_pult.mode_auv_real = msg_from_bort_for_planner.auvData.modeAUV_Real;

            message_to_pult.yaw = msg_from_bort_for_planner.auvData.ControlDataReal.yaw;
            message_to_pult.pitch = msg_from_bort_for_planner.auvData.ControlDataReal.pitch;
            message_to_pult.roll = msg_from_bort_for_planner.auvData.ControlDataReal.roll;
            message_to_pult.march = msg_from_bort_for_planner.auvData.ControlDataReal.march;
            message_to_pult.depth = msg_from_bort_for_planner.auvData.ControlDataReal.depth;
            message_to_pult.lag = msg_from_bort_for_planner.auvData.ControlDataReal.lag;

            message_to_pult.vma1 = msg_from_bort_for_planner.auvData.signalVMA_real.VMA1;
            message_to_pult.vma2 = msg_from_bort_for_planner.auvData.signalVMA_real.VMA2;
            message_to_pult.vma3 = msg_from_bort_for_planner.auvData.signalVMA_real.VMA3;
            message_to_pult.vma4 = msg_from_bort_for_planner.auvData.signalVMA_real.VMA4;
            message_to_pult.vma5 = msg_from_bort_for_planner.auvData.signalVMA_real.VMA5;
            message_to_pult.vma6 = msg_from_bort_for_planner.auvData.signalVMA_real.VMA6;

            message_to_pult.yaw_imu = msg_from_bort_for_planner.dataAH127C.yaw;
            message_to_pult.pitch_imu = msg_from_bort_for_planner.dataAH127C.pitch;
            message_to_pult.roll_imu = msg_from_bort_for_planner.dataAH127C.roll;
            message_to_pult.x_accel_imu = msg_from_bort_for_planner.dataAH127C.X_accel;
            message_to_pult.y_accel_imu = msg_from_bort_for_planner.dataAH127C.Y_accel;
            message_to_pult.z_accel_imu = msg_from_bort_for_planner.dataAH127C.Z_accel;
            message_to_pult.x_rate_imu = msg_from_bort_for_planner.dataAH127C.X_rate ;
            message_to_pult.y_rate_imu = msg_from_bort_for_planner.dataAH127C.Y_rate;
            message_to_pult.z_rate_imu = msg_from_bort_for_planner.dataAH127C.Z_rate;
            message_to_pult.x_magn_imu = msg_from_bort_for_planner.dataAH127C.X_magn;
            message_to_pult.y_magn_imu = msg_from_bort_for_planner.dataAH127C.Y_magn;
            message_to_pult.z_magn_imu = msg_from_bort_for_planner.dataAH127C.Z_magn;
            message_to_pult.quat[4] = msg_from_bort_for_planner.dataAH127C.quat[4];

            message_to_pult.temperature = msg_from_bort_for_planner.dataPressure.temperature;
            message_to_pult.depth_sensor = msg_from_bort_for_planner.dataPressure.depth;
            message_to_pult.pressure = msg_from_bort_for_planner.dataPressure.pressure;

            message_to_pult.error_code_uwb = msg_from_bort_for_planner.dataUWB.error_code;
            message_to_pult.connection_field_uwb = msg_from_bort_for_planner.dataUWB.connection_field;
            message_to_pult.location_x = msg_from_bort_for_planner.dataUWB.locationX;
            message_to_pult.location_y = msg_from_bort_for_planner.dataUWB.locationY;
            message_to_pult.distance_to_beacon[4] = msg_from_bort_for_planner.dataUWB.distanceToBeacon[4];
            message_to_pult.distance_to_agent[10] = msg_from_bort_for_planner.dataUWB.distanceToAgent[10];

            message_to_pult.start_calibration = msg_from_bort_for_planner.flagAH127C_bort.startCalibration;
            message_to_pult.end_calibration = msg_from_bort_for_planner.flagAH127C_bort.endCalibration;
            
            message_to_pult.id_mission = 2;
            message_to_pult.mission_status = 4;
            // message_to_pult = msg_from_bort_for_planner;
            // message_to_pult.ID_mission = 2;
            // message_to_pult.missionStatus = mission_Status::MODE_PERFOMED;
            // message_to_pult.checksum =checksum_i(&message_to_pult, sizeof(message_to_pult) - 4);
            // memcpy(&message_to_pult2,&message_to_pult,sizeof(message_to_pult)); 

            //message_to_pult.checksum = msg_from_bort_for_planner.checksum;
            //RCLCPP_INFO_STREAM(this->get_logger(), sizeof(message_to_pult2));
             
            

        } else {
            //message_to_bort.yaw_joy = atan2(y_v_SK_sv_x_y_goal,x_v_SK_sv_x_y_goal)*(180/M_PI);
            
            RCLCPP_INFO_STREAM(this->get_logger(), y_v_SK_sv_x_y_goal);
            RCLCPP_INFO_STREAM(this->get_logger(), x_v_SK_sv_x_y_goal);
            RCLCPP_INFO_STREAM(this->get_logger(), "^^^^^^^^^^^^^^^^^^^^^^^^");
            RCLCPP_INFO_STREAM(this->get_logger(), message_to_bort.yaw_joy);
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
            message_to_bort.march_joy = 40;
            message_to_bort.depth_joy= 0;
            message_to_bort.lag_joy= 0;
            message_to_bort.cs_mode = 2;
            message_to_bort.beacon_x[3] = msg_from_pult_for_planner.pultUWB.beacon_x[3];
            message_to_bort.beacon_y[3] = msg_from_pult_for_planner.pultUWB.beacon_y[3];
            message_to_bort.yaw_closed_real = 1;
            message_to_bort.pitch_closed_real = 0;
            message_to_bort.roll_closed_real = 0;
            message_to_bort.march_closed_real = 0;
            message_to_bort.depth_closed_real = 0;
            message_to_bort.lag_closed_real = 0;
            message_to_bort.mode_auv_selection = uint8_t(msg_from_pult_for_planner.cSMode);
            message_to_bort.power_mode = 2;
            message_to_bort.init_calibration = 0;
            message_to_bort.save_calibration = 0;
            message_to_bort.id_mission_auv = 1;
            message_to_bort.mission_command = uint8_t(msg_from_pult_for_planner.missionControl);
            message_to_bort.checksum_to_bort = 0;

            message_to_pult.sender_id = msg_from_bort_for_planner.headerSwap.senderID;
            message_to_pult.receiver_id = msg_from_bort_for_planner.headerSwap.receiverID;
            message_to_pult.msg_size = msg_from_bort_for_planner.headerSwap.msgSize;

            message_to_pult.mode_real = msg_from_bort_for_planner.auvData.modeReal;

            message_to_pult.yaw_contour = msg_from_bort_for_planner.auvData.controlReal.yaw;
            message_to_pult.pitch_contour = msg_from_bort_for_planner.auvData.controlReal.pitch;
            message_to_pult.roll_contour = msg_from_bort_for_planner.auvData.controlReal.roll;
            message_to_pult.march_contour = msg_from_bort_for_planner.auvData.controlReal.march;
            message_to_pult.depth_contour = msg_from_bort_for_planner.auvData.controlReal.depth;
            message_to_pult.lag_contour = msg_from_bort_for_planner.auvData.controlReal.lag;

            message_to_pult.mode_auv_real = msg_from_bort_for_planner.auvData.modeAUV_Real;

            message_to_pult.yaw = msg_from_bort_for_planner.auvData.ControlDataReal.yaw;
            message_to_pult.pitch = msg_from_bort_for_planner.auvData.ControlDataReal.pitch;
            message_to_pult.roll = msg_from_bort_for_planner.auvData.ControlDataReal.roll;
            message_to_pult.march = msg_from_bort_for_planner.auvData.ControlDataReal.march;
            message_to_pult.depth = msg_from_bort_for_planner.auvData.ControlDataReal.depth;
            message_to_pult.lag = msg_from_bort_for_planner.auvData.ControlDataReal.lag;

            message_to_pult.vma1 = msg_from_bort_for_planner.auvData.signalVMA_real.VMA1;
            message_to_pult.vma2 = msg_from_bort_for_planner.auvData.signalVMA_real.VMA2;
            message_to_pult.vma3 = msg_from_bort_for_planner.auvData.signalVMA_real.VMA3;
            message_to_pult.vma4 = msg_from_bort_for_planner.auvData.signalVMA_real.VMA4;
            message_to_pult.vma5 = msg_from_bort_for_planner.auvData.signalVMA_real.VMA5;
            message_to_pult.vma6 = msg_from_bort_for_planner.auvData.signalVMA_real.VMA6;

            message_to_pult.yaw_imu = msg_from_bort_for_planner.dataAH127C.yaw;
            message_to_pult.pitch_imu = msg_from_bort_for_planner.dataAH127C.pitch;
            message_to_pult.roll_imu = msg_from_bort_for_planner.dataAH127C.roll;
            message_to_pult.x_accel_imu = msg_from_bort_for_planner.dataAH127C.X_accel;
            message_to_pult.y_accel_imu = msg_from_bort_for_planner.dataAH127C.Y_accel;
            message_to_pult.z_accel_imu = msg_from_bort_for_planner.dataAH127C.Z_accel;
            message_to_pult.x_rate_imu = msg_from_bort_for_planner.dataAH127C.X_rate ;
            message_to_pult.y_rate_imu = msg_from_bort_for_planner.dataAH127C.Y_rate;
            message_to_pult.z_rate_imu = msg_from_bort_for_planner.dataAH127C.Z_rate;
            message_to_pult.x_magn_imu = msg_from_bort_for_planner.dataAH127C.X_magn;
            message_to_pult.y_magn_imu = msg_from_bort_for_planner.dataAH127C.Y_magn;
            message_to_pult.z_magn_imu = msg_from_bort_for_planner.dataAH127C.Z_magn;
            message_to_pult.quat[4] = msg_from_bort_for_planner.dataAH127C.quat[4];

            message_to_pult.temperature = msg_from_bort_for_planner.dataPressure.temperature;
            message_to_pult.depth_sensor = msg_from_bort_for_planner.dataPressure.depth;
            message_to_pult.pressure = msg_from_bort_for_planner.dataPressure.pressure;

            message_to_pult.error_code_uwb = msg_from_bort_for_planner.dataUWB.error_code;
            message_to_pult.connection_field_uwb = msg_from_bort_for_planner.dataUWB.connection_field;
            message_to_pult.location_x = msg_from_bort_for_planner.dataUWB.locationX;
            message_to_pult.location_y = msg_from_bort_for_planner.dataUWB.locationY;
            message_to_pult.distance_to_beacon[4] = msg_from_bort_for_planner.dataUWB.distanceToBeacon[4];
            message_to_pult.distance_to_agent[10] = msg_from_bort_for_planner.dataUWB.distanceToAgent[10];

            message_to_pult.start_calibration = msg_from_bort_for_planner.flagAH127C_bort.startCalibration;
            message_to_pult.end_calibration = msg_from_bort_for_planner.flagAH127C_bort.endCalibration;
            message_to_pult.id_mission = 2;
            message_to_pult.mission_status = 2;
        
        }
       
        // RCLCPP_INFO_STREAM(this->get_logger(), "To Pult data parameters staus and crc");
        // RCLCPP_INFO_STREAM(this->get_logger(), sizeof(message_to_pult.mission_status));
        // RCLCPP_INFO_STREAM(this->get_logger(), sizeof(message_to_pult.checksum));
        publisher_to_bort->publish(message_to_bort);
        publisher_to_pult->publish(message_to_pult);
    }

    void udp_message_pult_callback(udp_publisher::msg::ToBort const &msg_from_pult) {
    
    msg_from_pult_for_planner.controlData.yaw = msg_from_pult.yaw_joy;
    msg_from_pult_for_planner.controlData.pitch = msg_from_pult.pitch_joy;
    msg_from_pult_for_planner.controlData.roll = msg_from_pult.roll_joy;
    msg_from_pult_for_planner.controlData.march = msg_from_pult.march_joy;
    msg_from_pult_for_planner.controlData.depth = msg_from_pult.depth_joy;
    msg_from_pult_for_planner.controlData.lag = msg_from_pult.lag_joy;

    msg_from_pult_for_planner.cSMode = e_CSMode(msg_from_pult.cs_mode);

    msg_from_pult_for_planner.pultUWB.beacon_x[3] = msg_from_pult.beacon_x[3];
    msg_from_pult_for_planner.pultUWB.beacon_y[3] = msg_from_pult.beacon_y[3];

    msg_from_pult_for_planner.controlContoursFlags.yaw = msg_from_pult.yaw_closed_real;
    msg_from_pult_for_planner.controlContoursFlags.pitch = msg_from_pult.pitch_closed_real;
    msg_from_pult_for_planner.controlContoursFlags.roll = msg_from_pult.roll_closed_real;
    msg_from_pult_for_planner.controlContoursFlags.march = msg_from_pult.march_closed_real;
    msg_from_pult_for_planner.controlContoursFlags.depth = msg_from_pult.depth_closed_real;
    msg_from_pult_for_planner.controlContoursFlags.lag = msg_from_pult.lag_closed_real;

    msg_from_pult_for_planner.modeAUV_selection = msg_from_pult.mode_auv_selection;

    msg_from_pult_for_planner.pMode = power_Mode(msg_from_pult.power_mode);

    msg_from_pult_for_planner.flagAH127C_pult.initCalibration = msg_from_pult.init_calibration;
    msg_from_pult_for_planner.flagAH127C_pult.saveCalibration = msg_from_pult.save_calibration;

    msg_from_pult_for_planner.ID_mission_AUV = msg_from_pult.id_mission_auv;
    msg_from_pult_for_planner.missionControl = mission_Control(msg_from_pult.mission_command);

    msg_from_pult_for_planner.checksum = msg_from_pult.checksum_to_bort;
    }

    void udp_message_callback(udp_publisher::msg::FromBort const &msg_from_bort) {

    msg_from_bort_for_planner.headerSwap.senderID = msg_from_bort.sender_id;
    msg_from_bort_for_planner.headerSwap.receiverID = msg_from_bort.receiver_id;
    msg_from_bort_for_planner.headerSwap.msgSize = msg_from_bort.msg_size;

    msg_from_bort_for_planner.auvData.modeReal = msg_from_bort.mode_real;

    msg_from_bort_for_planner.auvData.controlReal.yaw = msg_from_bort.yaw_contour;
    msg_from_bort_for_planner.auvData.controlReal.pitch = msg_from_bort.pitch_contour;
    msg_from_bort_for_planner.auvData.controlReal.roll = msg_from_bort.roll_contour;
    msg_from_bort_for_planner.auvData.controlReal.march = msg_from_bort.march_contour;
    msg_from_bort_for_planner.auvData.controlReal.depth = msg_from_bort.depth_contour;
    msg_from_bort_for_planner.auvData.controlReal.lag = msg_from_bort.lag_contour;

    msg_from_bort_for_planner.auvData.modeAUV_Real = msg_from_bort.mode_auv_real;

    msg_from_bort_for_planner.auvData.ControlDataReal.yaw = msg_from_bort.yaw;
    msg_from_bort_for_planner.auvData.ControlDataReal.pitch = msg_from_bort.pitch;
    msg_from_bort_for_planner.auvData.ControlDataReal.roll = msg_from_bort.roll;
    msg_from_bort_for_planner.auvData.ControlDataReal.march = msg_from_bort.march;
    msg_from_bort_for_planner.auvData.ControlDataReal.depth = msg_from_bort.depth;
    msg_from_bort_for_planner.auvData.ControlDataReal.lag = msg_from_bort.lag;

    msg_from_bort_for_planner.auvData.signalVMA_real.VMA1 = msg_from_bort.vma1;
    msg_from_bort_for_planner.auvData.signalVMA_real.VMA2 = msg_from_bort.vma2;
    msg_from_bort_for_planner.auvData.signalVMA_real.VMA3 = msg_from_bort.vma3;
    msg_from_bort_for_planner.auvData.signalVMA_real.VMA4 = msg_from_bort.vma4;
    msg_from_bort_for_planner.auvData.signalVMA_real.VMA5 = msg_from_bort.vma5;
    msg_from_bort_for_planner.auvData.signalVMA_real.VMA6 = msg_from_bort.vma6;

    msg_from_bort_for_planner.dataAH127C.yaw = msg_from_bort.yaw_imu;
    msg_from_bort_for_planner.dataAH127C.pitch = msg_from_bort.pitch_imu;
    msg_from_bort_for_planner.dataAH127C.roll = msg_from_bort.roll_imu;
    msg_from_bort_for_planner.dataAH127C.X_accel = msg_from_bort.x_accel_imu;
    msg_from_bort_for_planner.dataAH127C.Y_accel = msg_from_bort.y_accel_imu;
    msg_from_bort_for_planner.dataAH127C.Z_accel = msg_from_bort.z_accel_imu;
    msg_from_bort_for_planner.dataAH127C.X_rate = msg_from_bort.x_rate_imu;
    msg_from_bort_for_planner.dataAH127C.Y_rate = msg_from_bort.y_rate_imu;
    msg_from_bort_for_planner.dataAH127C.Z_rate = msg_from_bort.z_rate_imu;
    msg_from_bort_for_planner.dataAH127C.X_magn = msg_from_bort.x_magn_imu;
    msg_from_bort_for_planner.dataAH127C.Y_magn = msg_from_bort.y_magn_imu;
    msg_from_bort_for_planner.dataAH127C.Z_magn = msg_from_bort.z_magn_imu;
    msg_from_bort_for_planner.dataAH127C.quat[4] = msg_from_bort.quat[4];

    msg_from_bort_for_planner.dataPressure.temperature = msg_from_bort.temperature;
    msg_from_bort_for_planner.dataPressure.depth = msg_from_bort.depth_sensor;
    msg_from_bort_for_planner.dataPressure.pressure = msg_from_bort.pressure;

    msg_from_bort_for_planner.dataUWB.error_code = msg_from_bort.error_code_uwb;
    msg_from_bort_for_planner.dataUWB.connection_field = msg_from_bort.connection_field_uwb;
    msg_from_bort_for_planner.dataUWB.locationX = msg_from_bort.location_x;
    msg_from_bort_for_planner.dataUWB.locationY = msg_from_bort.location_y;
    msg_from_bort_for_planner.dataUWB.distanceToBeacon[4] = msg_from_bort.distance_to_beacon[4];
    msg_from_bort_for_planner.dataUWB.distanceToAgent[10] = msg_from_bort.distance_to_agent[10];

    msg_from_bort_for_planner.flagAH127C_bort.startCalibration = msg_from_bort.start_calibration;
    msg_from_bort_for_planner.flagAH127C_bort.endCalibration = msg_from_bort.end_calibration;

    msg_from_bort_for_planner.ID_mission = msg_from_bort.id_mission;
    msg_from_bort_for_planner.missionStatus = mission_Status(msg_from_bort.mission_status);

    msg_from_bort_for_planner.checksum = 0;
    } 

private:
    rclcpp::TimerBase::SharedPtr timer_exit_to_point;
    rclcpp::TimerBase::SharedPtr timer_following;
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
    rclcpp::spin(std::make_shared<Control>());
    rclcpp::shutdown();
    return 0;
}

