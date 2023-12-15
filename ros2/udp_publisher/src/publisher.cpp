#include <chrono>
#include <memory>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/string.hpp"

#include "udp_publisher/udp_server.h"
#include "udp_publisher/udp_client.h"
#include "udp_publisher/protocols.h"

#include "udp_publisher/msg/from_bort.hpp"
#include "udp_publisher/msg/to_bort.hpp"

class MinimalPublisher : public rclcpp::Node {
public:
    // По факту это IP и ПОРТ на текущей тачке, так как тут будет сервер (хост), который только принимает
    static constexpr std::string_view RECEIVER_IP = "192.168.1.11"; 
    static constexpr unsigned short RECEIVER_PORT = 13051;
    // По факту это IP и ПОРТ на другой тачке, где запускается легаси борт QT как хост, а мы только шлем
    static constexpr std::string_view SENDER_IP = "192.168.1.11"; 
    static constexpr unsigned short SENDER_PORT = 13050;
    // IP и ПОРТ на текущей тачке, так как тут будет сервер, порт должен отличаться от другого приемника
    static constexpr std::string_view RECEIVER_IP_PULT = "192.168.1.11"; 
    static constexpr unsigned short RECEIVER_PORT_PULT = 13052;
    // IP и ПОРТ пульта(планировщика), на который мы можем слать обратную связь от ноды управления движением
    static constexpr std::string_view SENDER_IP_PULT = "192.168.1.173"; 
    static constexpr unsigned short SENDER_PORT_PULT = 13053;

    int flag_start_mission_1 = 0;
    int flag_start_mission_2 = 0;
    int flag_start_mission_3 = 0;

    MinimalPublisher()
    : Node("minimal_publisher")
    , publisher_(this->create_publisher<udp_publisher::msg::FromBort>("udp_message", 1))
    , publisher_pult(this->create_publisher<udp_publisher::msg::ToBort>("udp_message_pult", 1))
    , publisher_name_configure_file((this->create_publisher<std_msgs::msg::String>("name_config_file", 1)))
    , subscription_(this->create_subscription<udp_publisher::msg::ToBort>("planner_message", 1, std::bind(&MinimalPublisher::planner_msg_callback, this, std::placeholders::_1)))
    , subscription_to_pult(this->create_subscription<udp_publisher::msg::FromBort>("planner_feedback", 1, std::bind(&MinimalPublisher::planner_fbc_callback, this, std::placeholders::_1)))
    , receiver_from_bort(RECEIVER_IP, RECEIVER_PORT, std::bind(&MinimalPublisher::recv_callback, this, std::placeholders::_1), this->get_logger())
    , sender_to_bort(SENDER_IP, SENDER_PORT, this->get_logger())
    , receiver_from_pult(RECEIVER_IP_PULT, RECEIVER_PORT_PULT, std::bind(&MinimalPublisher::recv_callback_pult, this, std::placeholders::_1), this->get_logger())
    , sender_to_pult(SENDER_IP_PULT, SENDER_PORT_PULT, this->get_logger())
    {}

private:
    void planner_msg_callback(udp_publisher::msg::ToBort const &message) {  
        RCLCPP_INFO_STREAM(this->get_logger(), "Received from planner");
        ToBort rawMsg;

        rawMsg.controlData.yaw = message.yaw_joy;
        rawMsg.controlData.pitch = message.pitch_joy;
        rawMsg.controlData.roll = message.roll_joy;
        rawMsg.controlData.march = message.march_joy;
        rawMsg.controlData.depth = message.depth_joy;
        rawMsg.controlData.lag = message.lag_joy;
        rawMsg.cSMode = e_CSMode(message.cs_mode);
        rawMsg.pultUWB.beacon_x[3] = message.beacon_x[3];
        rawMsg.pultUWB.beacon_y[3] = message.beacon_y[3];
        rawMsg.controlContoursFlags.yaw = message.yaw_closed_real;
        rawMsg.controlContoursFlags.pitch = message.pitch_closed_real;
        rawMsg.controlContoursFlags.roll = message.roll_closed_real;
        rawMsg.controlContoursFlags.march = message.march_closed_real;
        rawMsg.controlContoursFlags.depth = message.depth_closed_real;
        rawMsg.controlContoursFlags.lag = message.lag_closed_real;
        rawMsg.modeAUV_selection = message.mode_auv_selection;
        rawMsg.pMode = power_Mode(message.power_mode);
        rawMsg.flagAH127C_pult.initCalibration = message.init_calibration;
        rawMsg.flagAH127C_pult.saveCalibration = message.save_calibration;
        rawMsg.ID_mission_AUV = message.id_mission_auv;
        rawMsg.missionControl = mission_Control(message.mission_command);
        rawMsg.checksum = message.checksum_to_bort;

        std::string sendMsg((char*)&rawMsg, sizeof(ToBort));
        sender_to_bort.send(sendMsg); 

        //здесь по идее нужно прописать случай окончания миссии, чтобы данные опять слались из борта
        //int flag_start_mission = 0;
    }

    uint checksum_i(const void * data, int size){ // function for checksum (uint)
        uint c = 0;
        for (int i = 0; i < size; ++i)
            c += ((const unsigned char*)data)[i];
        return ~(c + 1);
    }

    void planner_fbc_callback(udp_publisher::msg::FromBort const &message) { 
        FromBort rawMsg;

        rawMsg.headerSwap.senderID = message.sender_id;
        rawMsg.headerSwap.receiverID = message.receiver_id;
        rawMsg.headerSwap.msgSize = message.msg_size;

        rawMsg.auvData.modeReal = message.mode_real;

        rawMsg.auvData.controlReal.yaw = message.yaw_contour;
        rawMsg.auvData.controlReal.pitch = message.pitch_contour;
        rawMsg.auvData.controlReal.roll = message.roll_contour;
        rawMsg.auvData.controlReal.march = message.march_contour;
        rawMsg.auvData.controlReal.depth = message.depth_contour;
        rawMsg.auvData.controlReal.lag = message.lag_contour;

        rawMsg.auvData.modeAUV_Real = message.mode_auv_real;

        rawMsg.auvData.ControlDataReal.yaw = message.yaw;
        rawMsg.auvData.ControlDataReal.pitch = message.pitch;
        rawMsg.auvData.ControlDataReal.roll = message.roll;
        rawMsg.auvData.ControlDataReal.march = message.march;
        rawMsg.auvData.ControlDataReal.depth = message.depth;
        rawMsg.auvData.ControlDataReal.lag = message.lag;

        rawMsg.auvData.signalVMA_real.VMA1 = message.vma1;
        rawMsg.auvData.signalVMA_real.VMA2 = message.vma2;
        rawMsg.auvData.signalVMA_real.VMA3 = message.vma3;
        rawMsg.auvData.signalVMA_real.VMA4 = message.vma4;
        rawMsg.auvData.signalVMA_real.VMA5 = message.vma5;
        rawMsg.auvData.signalVMA_real.VMA6 = message.vma6;

        rawMsg.dataAH127C.yaw = message.yaw_imu;
        rawMsg.dataAH127C.pitch = message.pitch_imu;
        rawMsg.dataAH127C.roll = message.roll_imu;
        rawMsg.dataAH127C.X_accel = message.x_accel_imu;
        rawMsg.dataAH127C.Y_accel = message.y_accel_imu;
        rawMsg.dataAH127C.Z_accel = message.z_accel_imu;
        rawMsg.dataAH127C.X_rate = message.x_rate_imu;
        rawMsg.dataAH127C.Y_rate = message.y_rate_imu;
        rawMsg.dataAH127C.Z_rate = message.z_rate_imu;
        rawMsg.dataAH127C.X_magn = message.x_magn_imu;
        rawMsg.dataAH127C.Y_magn = message.y_magn_imu;
        rawMsg.dataAH127C.Z_magn = message.z_magn_imu;
        rawMsg.dataAH127C.quat[4] = message.quat[4];

        rawMsg.dataPressure.temperature = message.temperature;
        rawMsg.dataPressure.depth = message.depth_sensor;
        rawMsg.dataPressure.pressure = message.pressure;

        rawMsg.dataUWB.error_code = message.error_code_uwb;
        rawMsg.dataUWB.connection_field = message.connection_field_uwb;
        rawMsg.dataUWB.locationX = message.location_x;
        rawMsg.dataUWB.locationY = message.location_y;
        rawMsg.dataUWB.distanceToBeacon[4] = message.distance_to_beacon[4];
        rawMsg.dataUWB.distanceToAgent[10] = message.distance_to_agent[10];

        rawMsg.flagAH127C_bort.startCalibration = message.start_calibration;
        rawMsg.flagAH127C_bort.endCalibration = message.end_calibration;

        rawMsg.ID_mission = message.id_mission;
        rawMsg.missionStatus = mission_Status(message.mission_status);

        rawMsg.checksum =checksum_i(&rawMsg, sizeof(rawMsg) - 4);

        std::string sendMsg((char*)&rawMsg, sizeof(FromBort)); //???????????????????????      
        sender_to_pult.send(sendMsg); 
    }

    void recv_callback(std::string recv_msg) {
        RCLCPP_INFO_STREAM(this->get_logger(), "Received from bort");
        FromBort* rec = (FromBort*)recv_msg.data();
        auto message = udp_publisher::msg::FromBort();
    
        message.sender_id = rec->headerSwap.senderID;
        message.receiver_id = rec->headerSwap.receiverID;
        message.msg_size = rec->headerSwap.msgSize;

        message.mode_real = rec->auvData.modeReal;

        message.yaw_contour = rec->auvData.controlReal.yaw;
        message.pitch_contour = rec->auvData.controlReal.pitch;
        message.roll_contour = rec->auvData.controlReal.roll;
        message.march_contour = rec->auvData.controlReal.march;
        message.depth_contour = rec->auvData.controlReal.depth;
        message.lag_contour = rec->auvData.controlReal.lag;

        message.mode_auv_real = rec->auvData.modeAUV_Real;

        message.yaw = rec->auvData.ControlDataReal.yaw;
        message.pitch = rec->auvData.ControlDataReal.pitch;
        message.roll = rec->auvData.ControlDataReal.roll;
        message.march = rec->auvData.ControlDataReal.march;
        message.depth = rec->auvData.ControlDataReal.depth;
        message.lag = rec->auvData.ControlDataReal.lag;

        message.vma1 = rec->auvData.signalVMA_real.VMA1;
        message.vma2 = rec->auvData.signalVMA_real.VMA2;
        message.vma3 = rec->auvData.signalVMA_real.VMA3;
        message.vma4 = rec->auvData.signalVMA_real.VMA4;
        message.vma5 = rec->auvData.signalVMA_real.VMA5;
        message.vma6 = rec->auvData.signalVMA_real.VMA6;

        message.yaw_imu = rec->dataAH127C.yaw;
        message.pitch_imu = rec->dataAH127C.pitch;
        message.roll_imu = rec->dataAH127C.roll;
        message.x_accel_imu = rec->dataAH127C.X_accel;
        message.y_accel_imu = rec->dataAH127C.Y_accel;
        message.z_accel_imu = rec->dataAH127C.Z_accel;
        message.x_rate_imu = rec->dataAH127C.X_rate; 
        message.y_rate_imu = rec->dataAH127C.Y_rate;
        message.z_rate_imu = rec->dataAH127C.Z_rate;
        message.x_magn_imu = rec->dataAH127C.X_magn; 
        message.y_magn_imu = rec->dataAH127C.Y_magn;
        message.z_magn_imu = rec->dataAH127C.Z_magn;
        message.quat[4] = rec->dataAH127C.quat[4];

        message.temperature = rec->dataPressure.temperature;
        message.depth_sensor = rec->dataPressure.depth;
        message.pressure = rec->dataPressure.pressure;

        message.error_code_uwb = rec->dataUWB.error_code;
        message.connection_field_uwb = rec->dataUWB.connection_field;
        message.location_x = rec->dataUWB.locationX;
        message.location_y =  rec->dataUWB.locationY;
        message.distance_to_beacon[4] =  rec->dataUWB.distanceToBeacon[4];
        message.distance_to_agent[10] =  rec->dataUWB.distanceToAgent[10];

        message.start_calibration = rec->flagAH127C_bort.startCalibration;
        message.end_calibration = rec->flagAH127C_bort.endCalibration;

        message.id_mission = rec->ID_mission;
        message.mission_status = uint8_t(rec->missionStatus);

        message.checksum = rec->checksum;

        if (message.id_mission == 0 && message.mission_status == 0) {
            RCLCPP_INFO_STREAM(this->get_logger(), "eeeeeeeeerrrrrrrrrrrrrrroooooooooooooooooorrrrrrrrrrrrrr");
            sender_to_pult.send(recv_msg); 
        }

        // if (message.mode_real == 3) {
        // //автоматический режим
        // }

        publisher_->publish(message);
  
    }
 
    void recv_callback_pult(std::string recv_msg_pult) {
        RCLCPP_INFO_STREAM(this->get_logger(), "Received from pult");
        ToBort* rec = (ToBort*)recv_msg_pult.data();
        auto msg = udp_publisher::msg::ToBort();

        msg.yaw_joy = rec->controlData.yaw;
        msg.pitch_joy = rec->controlData.pitch;
        msg.roll_joy = rec->controlData.roll;
        msg.march_joy = rec->controlData.march;
        msg.depth_joy = rec->controlData.depth;
        msg.lag_joy = rec->controlData.lag;
        msg.cs_mode = uint8_t(rec->cSMode);
        msg.beacon_x[3] = rec->pultUWB.beacon_x[3];
        msg.beacon_y[3] = rec->pultUWB.beacon_y[3];
        msg.yaw_closed_real = rec->controlContoursFlags.yaw;
        msg.pitch_closed_real = rec->controlContoursFlags.pitch;
        msg.roll_closed_real = rec->controlContoursFlags.roll;
        msg.march_closed_real = rec->controlContoursFlags.march;
        msg.depth_closed_real = rec->controlContoursFlags.depth;
        msg.lag_closed_real = rec->controlContoursFlags.lag;
        msg.mode_auv_selection = rec->modeAUV_selection;
        msg.power_mode = uint8_t(rec->pMode);
        msg.init_calibration = rec->flagAH127C_pult.initCalibration;
        msg.save_calibration = rec->flagAH127C_pult.saveCalibration;
        msg.id_mission_auv = uint8_t(rec->ID_mission_AUV);
        msg.mission_command = uint8_t(rec->missionControl);
        msg.checksum_to_bort = rec->checksum;

        publisher_pult->publish(msg);

        if (msg.cs_mode == 2) { 
            if (msg.id_mission_auv == 1){  //выбрана миссия выхода в точку   
                if (msg.mission_command == 1  && flag_start_mission_2 == 0)   {
                    auto name = std_msgs::msg::String();
                    name.data = "go_to_point";
                    publisher_name_configure_file->publish(name);
                    flag_start_mission_2 = 1; 
                    RCLCPP_INFO_STREAM(this->get_logger(), "publish_Go_to_point_!!!!!!!!!!!!!!!!!!!!!");
                } 
                if (msg.mission_command == 4) { //выбрано завершение миссии
                    flag_start_mission_2 = 0; 
                }
        }
        }
        
        //flag_start_mission - для того, чтобы эта строка определения миссии постилась в топик единожды
        if (msg.cs_mode == 2) { //режим втоматический
            if (msg.id_mission_auv == 2)  {  //выбрана миссия следования    
                if (msg.mission_command == 1 && flag_start_mission_1 == 0)   { //команда старт нажата
                    auto name = std_msgs::msg::String();
                    name.data = "go_with_yaw_and_peleng";
                    publisher_name_configure_file->publish(name);
                    flag_start_mission_1 = 1; 
                    RCLCPP_INFO_STREAM(this->get_logger(), "publish_Go_with_yaw_and_peleng");
                }  
                if (msg.mission_command == 2 && flag_start_mission_1 == 1) {  //выбрана отмена миссии

                }   
                if (msg.mission_command == 3) { //выбрана приостановка миссии

                }
                if (msg.mission_command == 4) { //выбрано завершение миссии
                    flag_start_mission_1 = 0; 
                }
            }
        }

        if (msg.cs_mode == 2) { //режим втоматический
            if (msg.id_mission_auv == 3){ //выбрана миссия траектория
                if (msg.mission_command == 1  && flag_start_mission_3 == 0)   {
                    auto name = std_msgs::msg::String();
                    name.data = "go_along_trajectory";
                    publisher_name_configure_file->publish(name);
                    flag_start_mission_3 = 1; 
                    RCLCPP_INFO_STREAM(this->get_logger(), "go_along_trajectory");
                } 
                if (msg.mission_command == 4) { //выбрано завершение миссии
                    flag_start_mission_3 = 0; 
                }
        }
        }

        if (msg.id_mission_auv == 0 /* && (msg.mission_command == 0 || msg.mission_command == 4 )*/) {
            sender_to_bort.send(recv_msg_pult);   
            RCLCPP_INFO_STREAM(this->get_logger(), "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");        
        }
        
    }

private:
    rclcpp::Publisher<udp_publisher::msg::FromBort>::SharedPtr publisher_;
    rclcpp::Publisher<udp_publisher::msg::ToBort>::SharedPtr publisher_pult;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_name_configure_file;
    rclcpp::Subscription<udp_publisher::msg::ToBort>::SharedPtr subscription_;
    rclcpp::Subscription<udp_publisher::msg::FromBort>::SharedPtr subscription_to_pult;
    udp_publisher::UdpServer receiver_from_bort;
    udp_publisher::UdpClient sender_to_bort;
    udp_publisher::UdpServer receiver_from_pult;
    udp_publisher::UdpClient sender_to_pult;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MinimalPublisher>());
    rclcpp::shutdown();
    return 0;
}





// FromBort toPult;
//             toPult.headerSwap.senderID = message.sender_id;
//             toPult.headerSwap.receiverID = message.receiver_id;
//             toPult.headerSwap.msgSize = message.msg_size;

//             toPult.auvData.modeReal = message.mode_real;

//             toPult.auvData.controlReal.yaw = message.yaw_contour;
//             toPult.auvData.controlReal.pitch = message.pitch_contour;
//             toPult.auvData.controlReal.roll = message.roll_contour;
//             toPult.auvData.controlReal.march = message.march_contour;
//             toPult.auvData.controlReal.depth= message.depth_contour;
//             toPult.auvData.controlReal.lag = message.lag_contour;

//             toPult.auvData.modeAUV_Real = message.mode_auv_real;

//             toPult.auvData.ControlDataReal.yaw = message.yaw;
//             toPult.auvData.ControlDataReal.pitch = message.pitch;
//             toPult.auvData.ControlDataReal.roll = message.roll;
//             toPult.auvData.ControlDataReal.march = message.march;
//             toPult.auvData.ControlDataReal.depth = message.depth;
//             toPult.auvData.ControlDataReal.lag = message.lag;

//             toPult.auvData.signalVMA_real.VMA1 = message.vma1;
//             toPult.auvData.signalVMA_real.VMA2 = message.vma2;
//             toPult.auvData.signalVMA_real.VMA3 = message.vma3;
//             toPult.auvData.signalVMA_real.VMA4 = message.vma4;
//             toPult.auvData.signalVMA_real.VMA5 = message.vma5;
//             toPult.auvData.signalVMA_real.VMA6 = message.vma6;

//             toPult.dataAH127C.yaw = message.yaw_imu;
//             toPult.dataAH127C.pitch = message.pitch_imu;
//             toPult.dataAH127C.roll = message.roll_imu;
//             toPult.dataAH127C.X_accel = message.x_accel_imu;
//             toPult.dataAH127C.Y_accel = message.y_accel_imu;
//             toPult.dataAH127C.Z_accel = message.z_accel_imu;
//             toPult.dataAH127C.X_rate = message.x_rate_imu;
//             toPult.dataAH127C.Y_rate = message.y_rate_imu;
//             toPult.dataAH127C.Z_rate = message.z_rate_imu;
//             toPult.dataAH127C.X_magn = message.x_magn_imu;
//             toPult.dataAH127C.Y_magn = message.y_magn_imu;
//             toPult.dataAH127C.Z_magn = message.z_magn_imu;
//             toPult.dataAH127C.quat[4] = message.quat[4];

//             toPult.dataPressure.temperature = message.temperature;
//             toPult.dataPressure.depth = message.depth_sensor;
//             toPult.dataPressure.pressure = message.pressure;

//             toPult.dataUWB.locationX = message.location_x;
//             toPult.dataUWB.locationY = message.location_y;
//             toPult.dataUWB.distanceToBeacon[4] = message.distance_to_beacon[4];
//             toPult.dataUWB.distanceToAgent[10] = message.distance_to_agent[10];

//             toPult.flagAH127C_bort.startCalibration = message.start_calibration;
//             toPult.flagAH127C_bort.endCalibration = message.end_calibration;

//             toPult.checksum = message.checksum;

//             std::string sendMsg((char*)&toPult, sizeof(FromBort));
//             sender_to_pult.send(sendMsg); 