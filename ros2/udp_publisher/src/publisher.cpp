#include <chrono>
#include <memory>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include "udp_publisher/udp_server.h"
#include "udp_publisher/udp_client.h"
#include "udp_publisher/protocols.h"

#include "udp_publisher/msg/from_bort.hpp"
#include "udp_publisher/msg/to_bort.hpp"

class MinimalPublisher : public rclcpp::Node {
public:
    // По факту это IP и ПОРТ на текущей тачке, так как тут будет сервер (хост), который только принимает
    static constexpr std::string_view RECIEVER_IP = "127.0.0.1"; 
    static constexpr unsigned short RECIEVER_PORT = 13051;
    // По факту это IP и ПОРТ на другой тачке, где запускается легаси борт QT как хост, а мы только шлем
    static constexpr std::string_view SENDER_IP = "127.0.0.1"; 
    static constexpr unsigned short SENDER_PORT = 13050;

    MinimalPublisher()
    : Node("minimal_publisher")
    , publisher_(this->create_publisher<udp_publisher::msg::FromBort>("udp_message", 1))
    , subscription_(this->create_subscription<udp_publisher::msg::ToBort>("planner_message", 1, std::bind(&MinimalPublisher::planner_msg_callback, this, std::placeholders::_1)))
    , reciever_(RECIEVER_IP, RECIEVER_PORT, std::bind(&MinimalPublisher::recv_callback, this, std::placeholders::_1), this->get_logger())
    , sender_(SENDER_IP, SENDER_PORT, this->get_logger())
    {}

private:
    void planner_msg_callback(udp_publisher::msg::ToBort &message) {  //убрала const, потому что ругался
        RCLCPP_INFO_STREAM(this->get_logger(), "Received from planner");
        udp_publisher::msg::ToBort* snd = &message;
        std::string sendMsg = std::to_string(snd->yaw_joy) + std::to_string(snd->pitch_joy) + 
        std::to_string(snd->roll_joy) + std::to_string(snd->march_joy) + std::to_string(snd->depth_joy) + 
        std::to_string(snd->lag_joy) + std::to_string(snd->cs_mode) + std::to_string(snd->beacon_x[3]) + 
        std::to_string(snd->beacon_y[3])  + std::to_string(snd->yaw_closed_real) + std::to_string(snd->pitch_closed_real) + 
        std::to_string(snd->roll_closed_real) + std::to_string(snd->march_closed_real) + std::to_string(snd->depth_closed_real) + 
        std::to_string(snd->lag_closed_real) + std::to_string(snd->mode_auv_selection) + std::to_string(snd->power_mode) + 
        std::to_string(snd->init_calibration) + std::to_string(snd->save_calibration) + std::to_string(snd->checksum_to_bort);

        sender_.send(sendMsg);
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

        message.location_x = rec->dataUWB.locationX;
        message.location_y =  rec->dataUWB.locationY;
        message.distance_to_beacon[4] =  rec->dataUWB.distanceToBeacon[4];
        message.distance_to_agent[10] =  rec->dataUWB.distanceToAgent[10];

        message.start_calibration = rec->flagAH127C_bort.startCalibration;
        message.end_calibration = rec->flagAH127C_bort.endCalibration;

        message.checksum = rec->checksum;

        publisher_->publish(message);
    }
 
    // void timer_callback() {
    //     //здесь я так понимаю нужно также заполнить посылку на борт
    //     // В QT было: sendSocket->writeDatagram((char *)&send_data, sizeof(send_data),m_ip_sender, m_port_sender);

    //     // assert(!last_planner_message_.empty());
    //     //std::cout << last_planner_message_;
    //     udp_client_.send(last_planner_message_);
        
    // }

private:
    rclcpp::Publisher<udp_publisher::msg::FromBort>::SharedPtr publisher_;
    rclcpp::Subscription<udp_publisher::msg::ToBort>::SharedPtr subscription_;
    udp_publisher::UdpServer reciever_;
    udp_publisher::UdpClient sender_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MinimalPublisher>());
    rclcpp::shutdown();
    return 0;
}
