#ifndef UDP_CLIENT
#define UDP_CLIENT

#include <thread>

#include <boost/asio.hpp>
#include <boost/array.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/rclcpp.hpp>

namespace udp_publisher {

using boost::asio::ip::udp;
using boost::asio::ip::address;

class UdpClient {
public:
    UdpClient(const std::string_view host, const unsigned short port, rclcpp::Logger logger): 
        logger_(logger)
        , remote_endpoint_(address::from_string(host.data()), port)
        , worker_(boost::asio::make_work_guard(io_service_))
    {
        std::thread([&](){ run(); }).detach();
    }

    void send(const std::string &data) {
        io_service_.post(std::bind(&UdpClient::do_send, this, data));
        
    }

private:
    void run() {
        socket_.open(udp::v4());
        RCLCPP_INFO(logger_, "Sending ready");
        io_service_.run();
        RCLCPP_INFO(logger_, "Sending exit");
    }

    void do_send(const std::string &data) {
        if (!socket_.is_open()) {
            RCLCPP_ERROR(logger_, "Can't send - socket is not open!");
        }
        socket_.async_send_to(boost::asio::buffer(data), remote_endpoint_,
            std::bind(&UdpClient::handle_send, this, std::placeholders::_1, std::placeholders::_2));
    }

    void handle_send(const boost::system::error_code& error, std::size_t bytes_transferred) {
        if(error) {
            RCLCPP_ERROR_STREAM(logger_, "Send failed: " << error.message());
            return;
        }
        RCLCPP_INFO_STREAM(logger_, "Send " << bytes_transferred  << " bytes!");
    }

private:
    rclcpp::Logger logger_;
    udp::endpoint remote_endpoint_;
    boost::asio::io_service io_service_;
    boost::asio::executor_work_guard<boost::asio::io_context::executor_type> worker_;
    udp::socket socket_{io_service_};
};

}  // namespace udp_publisher

#endif  // UDP_CLIENT