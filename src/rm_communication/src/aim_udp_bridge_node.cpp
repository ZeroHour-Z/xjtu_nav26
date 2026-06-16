#include "std_msgs/msg/u_int8_multi_array.hpp"

#include <array>
#include <atomic>
#include <arpa/inet.h>
#include <chrono>
#include <cstring>
#include <errno.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <stdexcept>
#include <string>
#include <sys/socket.h>
#include <thread>
#include <unistd.h>
#include <vector>

namespace {

constexpr size_t kFrameSize = 64;
constexpr uint8_t kAimHeader = 0x71;
constexpr uint8_t kAimTail = 0x4C;
constexpr size_t kAimAutoTailOffset = 54;
constexpr size_t kAimWmTailOffset = 63;

static sockaddr_in makeAddr(const std::string& host, int port) {
    sockaddr_in addr {};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(static_cast<uint16_t>(port));
    if (host.empty() || host == "0.0.0.0") {
        addr.sin_addr.s_addr = htonl(INADDR_ANY);
    } else if (::inet_pton(AF_INET, host.c_str(), &addr.sin_addr) != 1) {
        throw std::runtime_error("invalid IPv4 address: " + host);
    }
    return addr;
}

static bool validAimTxPacket(const std::vector<uint8_t>& data) {
    return data.size() == kFrameSize && data.front() == kAimHeader &&
           (data[kAimAutoTailOffset] == kAimTail || data[kAimWmTailOffset] == kAimTail);
}

static bool validAimRxPacket(const std::vector<uint8_t>& data) {
    return data.size() == kFrameSize && data.front() == kAimHeader;
}

} // namespace

class AimUdpBridgeNode: public rclcpp::Node {
public:
    AimUdpBridgeNode(): Node("aim_udp_bridge_node") {
        bind_host_ = this->declare_parameter<std::string>("bind_host", "0.0.0.0");
        peer_host_ = this->declare_parameter<std::string>("peer_host", "127.0.0.1");
        aim_to_nav_port_ = this->declare_parameter<int>("aim_to_nav_port", 47001);
        nav_to_aim_port_ = this->declare_parameter<int>("nav_to_aim_port", 47002);

        tx_pub_ = this->create_publisher<std_msgs::msg::UInt8MultiArray>(
            "/rm_comm/aim_tx_packet",
            rclcpp::QoS(10).reliable()
        );
        rx_sub_ = this->create_subscription<std_msgs::msg::UInt8MultiArray>(
            "/rm_comm/aim_rx_packet",
            rclcpp::QoS(10).reliable(),
            std::bind(&AimUdpBridgeNode::onAimRxPacket, this, std::placeholders::_1)
        );

        openSocket();
        running_.store(true);
        recv_thread_ = std::thread(&AimUdpBridgeNode::recvLoop, this);

        RCLCPP_INFO(
            this->get_logger(),
            "aim_udp_bridge_node started: UDP %s:%d -> /rm_comm/aim_tx_packet, "
            "/rm_comm/aim_rx_packet -> %s:%d",
            bind_host_.c_str(),
            aim_to_nav_port_,
            peer_host_.c_str(),
            nav_to_aim_port_
        );
    }

    ~AimUdpBridgeNode() override {
        running_.store(false);
        if (fd_ >= 0) {
            ::close(fd_);
            fd_ = -1;
        }
        if (recv_thread_.joinable()) {
            recv_thread_.join();
        }
    }

private:
    void openSocket() {
        fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
        if (fd_ < 0) {
            throw std::runtime_error(std::string("socket failed: ") + std::strerror(errno));
        }

        const int enable = 1;
        ::setsockopt(fd_, SOL_SOCKET, SO_REUSEADDR, &enable, sizeof(enable));

        sockaddr_in bind_addr = makeAddr(bind_host_, aim_to_nav_port_);
        if (::bind(fd_, reinterpret_cast<sockaddr*>(&bind_addr), sizeof(bind_addr)) < 0) {
            const std::string error = std::strerror(errno);
            ::close(fd_);
            fd_ = -1;
            throw std::runtime_error("bind UDP port failed: " + error);
        }

        int flags = ::fcntl(fd_, F_GETFL, 0);
        if (flags >= 0) {
            ::fcntl(fd_, F_SETFL, flags | O_NONBLOCK);
        }

        peer_addr_ = makeAddr(peer_host_, nav_to_aim_port_);
    }

    void recvLoop() {
        std::array<uint8_t, kFrameSize> buffer {};

        while (running_.load()) {
            ssize_t n = ::recv(fd_, buffer.data(), buffer.size(), 0);
            if (n < 0) {
                if (errno == EINTR) {
                    continue;
                }
                if (errno == EAGAIN || errno == EWOULDBLOCK) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(1));
                    continue;
                }
                RCLCPP_ERROR(this->get_logger(), "UDP recv failed: %s", std::strerror(errno));
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                continue;
            }

            if (static_cast<size_t>(n) != kFrameSize) {
                RCLCPP_WARN_THROTTLE(
                    this->get_logger(),
                    *this->get_clock(),
                    2000,
                    "aim UDP packet size %ld != %zu, drop",
                    n,
                    kFrameSize
                );
                continue;
            }

            std_msgs::msg::UInt8MultiArray msg;
            msg.data.assign(buffer.begin(), buffer.end());
            if (!validAimTxPacket(msg.data)) {
                RCLCPP_WARN_THROTTLE(
                    this->get_logger(),
                    *this->get_clock(),
                    2000,
                    "aim UDP packet bad framing head=0x%02X tail@54=0x%02X tail@63=0x%02X, drop",
                    (unsigned)msg.data.front(),
                    (unsigned)msg.data[kAimAutoTailOffset],
                    (unsigned)msg.data[kAimWmTailOffset]
                );
                continue;
            }

            tx_pub_->publish(msg);
            RCLCPP_INFO_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                1000,
                "aim UDP -> /rm_comm/aim_tx_packet head=0x%02X tail@54=0x%02X tail@63=0x%02X",
                (unsigned)msg.data.front(),
                (unsigned)msg.data[kAimAutoTailOffset],
                (unsigned)msg.data[kAimWmTailOffset]
            );
        }
    }

    void onAimRxPacket(const std_msgs::msg::UInt8MultiArray::SharedPtr msg) {
        if (!validAimRxPacket(msg->data)) {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                2000,
                "aim_rx_packet bad size/header, drop"
            );
            return;
        }

        ssize_t n = ::sendto(
            fd_,
            msg->data.data(),
            msg->data.size(),
            0,
            reinterpret_cast<sockaddr*>(&peer_addr_),
            sizeof(peer_addr_)
        );
        if (n != static_cast<ssize_t>(msg->data.size())) {
            RCLCPP_ERROR(this->get_logger(), "UDP send failed: %s", std::strerror(errno));
        }
    }

    std::string bind_host_;
    std::string peer_host_;
    int aim_to_nav_port_ { 47001 };
    int nav_to_aim_port_ { 47002 };

    rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr tx_pub_;
    rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr rx_sub_;

    std::atomic<bool> running_ { false };
    std::thread recv_thread_;
    int fd_ { -1 };
    sockaddr_in peer_addr_ {};
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AimUdpBridgeNode>());
    rclcpp::shutdown();
    return 0;
}
