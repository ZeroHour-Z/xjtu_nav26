#include "packet_utils.hpp"
#include "protocol.h"
#include "std_msgs/msg/u_int8_multi_array.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstring>
#include <errno.h>
#include <fcntl.h>
#include <mutex>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <sys/ioctl.h>
#include <termios.h>
#include <thread>
#include <unistd.h>

using namespace std::chrono_literals;

// 本节点直接操作串口设备：是 PC 端唯一持有 /dev/ttyACM* 的进程，
// 同时承载两套协议（nav 与 aimbot）的多路复用 broker：
// - 参数：port=/dev/ttyACM0, baud=115200
// - 订阅: /rm_comm/tx_packet      (handler 打包好的 64B navInfo_t,    head=0x72 tail=0x4D)
// - 订阅: /rm_comm/aim_tx_packet  (aimbot 打包好的 64B,                head=0x71 tail=0x4C)
// - 发布: /rm_comm/rx_packet      (从串口读取的 navCommand_t,          head=0x72 tail=0x21)
// - 发布: /rm_comm/aim_rx_packet  (从串口读取的 aimbot 上行 64B,       head=0x71 tail=0x4C)

namespace {

static speed_t to_baud_constant(int baud) {
    switch (baud) {
        case 9600:
            return B9600;
        case 19200:
            return B19200;
        case 38400:
            return B38400;
        case 57600:
            return B57600;
        case 115200:
            return B115200;
        case 230400:
            return B230400;
        default:
            return B115200;
    }
}

// 64 字节定长帧。两套协议长度相同，仅靠帧头/帧尾区分。
constexpr size_t kFrameSize = 64;

constexpr uint8_t kNavHeader = 0x72;
constexpr uint8_t kNavTailRx = 0x21; // navCommand_t 帧尾（电控 -> PC）
constexpr uint8_t kNavTailTx = 0x4D; // navInfo_t 帧尾   （PC   -> 电控）

constexpr uint8_t kAimHeader = 0x71;
constexpr uint8_t kAimTail = 0x4C;
// aim 两种 union 视图的 tail 位置不同：
// MessData_AutoAim tail 在 byte[54]，MessData_WM tail 在 byte[63]。
constexpr size_t kAimAutoTailOffset = 54;
constexpr size_t kAimWmTailOffset = 63;

static_assert(sizeof(navCommand_t) == kFrameSize, "navCommand_t must be 64 bytes");
static_assert(sizeof(navInfo_t) == kFrameSize, "navInfo_t must be 64 bytes");

} // namespace

class SerialRwNode: public rclcpp::Node {
public:
    SerialRwNode(): Node("serial_rw_node") {
        port_ = this->declare_parameter<std::string>("port", "/dev/ttyACM0");
        baud_ = this->declare_parameter<int>("baud", 115200);
        reopen_interval_ms_ = this->declare_parameter<int>("reopen_interval_ms", 500);
        rx_diagnostics_ = this->declare_parameter<bool>("rx_diagnostics", false);
        nav_rx_log_period_ms_ = this->declare_parameter<int>("nav_rx_log_period_ms", 1000);
        aim_rx_log_period_ms_ = this->declare_parameter<int>("aim_rx_log_period_ms", 5000);
        double read_loop_hz = this->declare_parameter<double>("read_loop_hz", 200.0);
        if (read_loop_hz <= 0.0) {
            read_loop_hz = 1.0;
        }
        auto desired_period = std::chrono::duration<double>(1.0 / read_loop_hz);
        idle_sleep_duration_ =
            std::chrono::duration_cast<std::chrono::milliseconds>(desired_period);
        if (idle_sleep_duration_.count() <= 0) {
            idle_sleep_duration_ = std::chrono::milliseconds(1);
        }

        // nav 链路
        tx_sub_ = this->create_subscription<std_msgs::msg::UInt8MultiArray>(
            "/rm_comm/tx_packet",
            rclcpp::QoS(10).reliable(),
            std::bind(&SerialRwNode::onNavTxPacket, this, std::placeholders::_1)
        );

        rx_pub_ = this->create_publisher<std_msgs::msg::UInt8MultiArray>(
            "/rm_comm/rx_packet",
            rclcpp::QoS(10).reliable()
        );

        // aim 链路
        aim_tx_sub_ = this->create_subscription<std_msgs::msg::UInt8MultiArray>(
            "/rm_comm/aim_tx_packet",
            rclcpp::QoS(10).reliable(),
            std::bind(&SerialRwNode::onAimTxPacket, this, std::placeholders::_1)
        );

        aim_rx_pub_ = this->create_publisher<std_msgs::msg::UInt8MultiArray>(
            "/rm_comm/aim_rx_packet",
            rclcpp::QoS(10).reliable()
        );

        running_.store(true);

        read_thread_ = std::thread(&SerialRwNode::readLoop, this);

        RCLCPP_INFO(
            this->get_logger(),
            "serial_rw_node started, port:%s, baud:%d (multi-proto: nav 0x72/0x21, aim 0x71/0x4C)",
            port_.c_str(),
            baud_
        );
    }

    ~SerialRwNode() override {
        running_.store(false);
        closeSerial();
        if (read_thread_.joinable()) {
            read_thread_.join();
        }
    }

private:
    // nav 下行：navInfo_t 严格长度校验
    void onNavTxPacket(const std_msgs::msg::UInt8MultiArray::SharedPtr msg) {
        if (msg->data.size() != kFrameSize) {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                2000,
                "[nav] tx_packet size %zu != %zu, drop",
                msg->data.size(),
                kFrameSize
            );
            return;
        }
        if (msg->data.front() != kNavHeader || msg->data.back() != kNavTailTx) {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                2000,
                "[nav] tx_packet bad framing head=0x%02X tail=0x%02X, drop",
                (unsigned)msg->data.front(),
                (unsigned)msg->data.back()
            );
            return;
        }
        writeBytes(msg->data.data(), msg->data.size(), "nav");
    }

    // aim 下行：64B + head=0x71 + tail=0x4C
    void onAimTxPacket(const std_msgs::msg::UInt8MultiArray::SharedPtr msg) {
        if (msg->data.size() != kFrameSize) {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                2000,
                "[aim] tx_packet size %zu != %zu, drop",
                msg->data.size(),
                kFrameSize
            );
            return;
        }
        const bool has_aim_tail = msg->data[kAimAutoTailOffset] == kAimTail ||
                                  msg->data[kAimWmTailOffset] == kAimTail;
        if (msg->data.front() != kAimHeader || !has_aim_tail) {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                2000,
                "[aim] tx_packet bad framing head=0x%02X tail@54=0x%02X tail@63=0x%02X, drop",
                (unsigned)msg->data.front(),
                (unsigned)msg->data[kAimAutoTailOffset],
                (unsigned)msg->data[kAimWmTailOffset]
            );
            return;
        }
        RCLCPP_DEBUG_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            1000,
            "[aim] TX accepted, writing 64B to serial tail@54=0x%02X tail@63=0x%02X",
            (unsigned)msg->data[kAimAutoTailOffset],
            (unsigned)msg->data[kAimWmTailOffset]
        );
        writeBytes(msg->data.data(), msg->data.size(), "aim");
    }

    // 多线程下统一写串口入口（onNavTxPacket / onAimTxPacket 共用）
    void writeBytes(const uint8_t* data, size_t size, const char* tag) {
        const int fd_snapshot = currentFd();
        if (fd_snapshot < 0) {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                2000,
                "[%s] serial not open; drop tx",
                tag
            );
            return;
        }

        std::lock_guard<std::mutex> lock(write_mutex_);
        size_t remaining = size;
        while (remaining > 0) {
            ssize_t n = ::write(fd_snapshot, data, remaining);
            if (n < 0) {
                if (errno == EINTR)
                    continue;
                RCLCPP_ERROR(
                    this->get_logger(),
                    "[%s] write error: %s",
                    tag,
                    std::strerror(errno)
                );
                closeSerial();
                return;
            }
            remaining -= static_cast<size_t>(n);
            data += n;
        }
    }

    void readLoop() {
        while (running_.load()) {
            int fd_snapshot = currentFd();
            if (fd_snapshot < 0) {
                if (!openSerial()) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(reopen_interval_ms_));
                    continue;
                }
                continue;
            }

            uint8_t buf[512];
            ssize_t n = ::read(fd_snapshot, buf, sizeof(buf));
            RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "read n: %ld", n);

            if (n > 0) {
                if (rx_diagnostics_) {
                    RCLCPP_INFO_THROTTLE(
                        this->get_logger(),
                        *this->get_clock(),
                        1000,
                        "serial RX raw: %ld bytes, first bytes: %s",
                        static_cast<long>(n),
                        formatBytes(buf, static_cast<size_t>(n), 8).c_str()
                    );
                }
                rx_buffer_.insert(rx_buffer_.end(), buf, buf + n);

                // 防止缓冲区无限增长（正常情况下 < 几百字节）
                constexpr size_t kMaxBufferSize = 4096;
                if (rx_buffer_.size() > kMaxBufferSize) {
                    RCLCPP_WARN_THROTTLE(
                        this->get_logger(), *this->get_clock(), 2000,
                        "rx_buffer_ overflow (%zu bytes), clearing", rx_buffer_.size());
                    rx_buffer_.clear();
                    continue;
                }

                parseAndDispatch();
                continue;
            }

            if (n == 0) {
                if (rx_diagnostics_) {
                    RCLCPP_INFO_THROTTLE(
                        this->get_logger(),
                        *this->get_clock(),
                        3000,
                        "serial open but no RX bytes on %s",
                        port_.c_str()
                    );
                }
                std::this_thread::sleep_for(idle_sleep_duration_);
                continue;
            }

            // n < 0
            if (errno == EINTR) {
                continue;
            }
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                std::this_thread::sleep_for(idle_sleep_duration_);
                continue;
            }

            RCLCPP_WARN(this->get_logger(), "read error: %s", std::strerror(errno));
            closeSerial();
            std::this_thread::sleep_for(std::chrono::milliseconds(reopen_interval_ms_));
        }
    }

    // 在 rx_buffer_ 中找下一个 nav/aim 候选帧头，分别发布。
    // nav 上行兼容电控未填 frame_tail 的情况：只按 0x72 + 64B 定长解析。
    void parseAndDispatch() {
        while (true) {
            // 找下一个等于 nav 或 aim 帧头的字节
            auto it = std::find_if(
                rx_buffer_.begin(),
                rx_buffer_.end(),
                [](uint8_t b) { return b == kNavHeader || b == kAimHeader; }
            );
            if (it == rx_buffer_.end()) {
                if (rx_diagnostics_ && !rx_buffer_.empty()) {
                    RCLCPP_WARN_THROTTLE(
                        this->get_logger(),
                        *this->get_clock(),
                        1000,
                        "RX drop %zu bytes: no known header 0x72(nav) or 0x71(aim), first bytes: %s",
                        rx_buffer_.size(),
                        formatBytes(rx_buffer_.data(), rx_buffer_.size(), 8).c_str()
                    );
                }
                rx_buffer_.clear();
                return;
            }
            if (it != rx_buffer_.begin()) {
                if (rx_diagnostics_) {
                    const size_t drop_count =
                        static_cast<size_t>(std::distance(rx_buffer_.begin(), it));
                    RCLCPP_WARN_THROTTLE(
                        this->get_logger(),
                        *this->get_clock(),
                        1000,
                        "RX resync: dropped %zu bytes before header 0x%02X",
                        drop_count,
                        static_cast<unsigned>(*it)
                    );
                }
                rx_buffer_.erase(rx_buffer_.begin(), it);
            }
            if (rx_buffer_.size() < kFrameSize) {
                return; // 等待数据
            }

            const uint8_t header = rx_buffer_.front();

            bool ok = false;
            if (header == kNavHeader) {
                ok = true;
            } else if (header == kAimHeader) {
                // 兼容性：aim 两种结构 tail 分别在 byte[54]/byte[63]；
                // 若电控未填 tail，则用
                // "下一字节是另一个合法帧头(0x71/0x72)" 作为 64B 边界校验。
                // 这样既能恢复链路，又能在帧间错位时拒收以重同步。
                if (rx_buffer_[kAimAutoTailOffset] == kAimTail ||
                    rx_buffer_[kAimWmTailOffset] == kAimTail) {
                    ok = true;
                } else if (rx_buffer_.size() >= kFrameSize + 1) {
                    const uint8_t next = rx_buffer_[kFrameSize];
                    if (next == kNavHeader || next == kAimHeader) {
                        ok = true;
                        RCLCPP_WARN_ONCE(
                            this->get_logger(),
                            "[aim] tail byte[54]=0x%02X byte[63]=0x%02X != 0x4C; accepting based on "
                            "64B alignment (next head=0x%02X). MCU should write "
                            "frame tail for stricter validation.",
                            (unsigned)rx_buffer_[kAimAutoTailOffset],
                            (unsigned)rx_buffer_[kAimWmTailOffset],
                            (unsigned)next
                        );
                    }
                } else {
                    // 还没收到下一帧首字节，等待更多数据再判断
                    return;
                }
            }

            if (!ok) {
                if (rx_diagnostics_) {
                    if (header == kNavHeader) {
                        RCLCPP_WARN_THROTTLE(
                            this->get_logger(),
                            *this->get_clock(),
                            1000,
                            "[nav] candidate rejected: head=0x72 byte[63]=0x%02X expected tail=0x21, first bytes: %s",
                            static_cast<unsigned>(rx_buffer_[kFrameSize - 1]),
                            formatBytes(rx_buffer_.data(), rx_buffer_.size(), 8).c_str()
                        );
                    } else if (header == kAimHeader) {
                        RCLCPP_WARN_THROTTLE(
                            this->get_logger(),
                            *this->get_clock(),
                            1000,
                            "[aim] candidate rejected: byte[54]=0x%02X byte[63]=0x%02X expected 0x4C",
                            static_cast<unsigned>(rx_buffer_[kAimAutoTailOffset]),
                            static_cast<unsigned>(rx_buffer_[kAimWmTailOffset])
                        );
                    }
                }
                // 当前候选帧头与帧尾不匹配 -> 丢弃 1 字节继续重同步
                rx_buffer_.erase(rx_buffer_.begin());
                continue;
            }

            std_msgs::msg::UInt8MultiArray out_msg;
            out_msg.data.assign(rx_buffer_.begin(), rx_buffer_.begin() + kFrameSize);

            if (header == kNavHeader) {
                if (out_msg.data.back() != kNavTailRx) {
                    RCLCPP_WARN_ONCE(
                        this->get_logger(),
                        "[nav] RX frame_tail byte[63]=0x%02X != 0x21; accepting 64B frame by header only and normalizing tail for handler_node.",
                        static_cast<unsigned>(out_msg.data.back())
                    );
                    out_msg.data.back() = kNavTailRx;
                }
                navCommand_t n_data;
                std::memcpy(&n_data, out_msg.data.data(), kFrameSize);
                RCLCPP_INFO_THROTTLE(
                    this->get_logger(),
                    *this->get_clock(),
                    nav_rx_log_period_ms_,
                    "[nav] RX color:%d state:%d motion:%d region:%d hp:%d bullet:%d enemy:(%.2f,%.2f) target:(%.2f,%.2f)",
                    (int)n_data.color,
                    (int)n_data.eSentryState,
                    (int)n_data.motion_allowed,
                    (int)n_data.patrol_region,
                    (int)n_data.hp_remain,
                    (int)n_data.bullet_remain,
                    n_data.enemy_x,
                    n_data.enemy_y,
                    n_data.target_x,
                    n_data.target_y
                );
                rx_pub_->publish(out_msg);
            } else {
                RCLCPP_INFO_THROTTLE(
                    this->get_logger(),
                    *this->get_clock(),
                    aim_rx_log_period_ms_,
                    "[aim] RX 64B head=0x71 tail=0x4C"
                );
                aim_rx_pub_->publish(out_msg);
            }

            rx_buffer_.erase(rx_buffer_.begin(), rx_buffer_.begin() + kFrameSize);
        }
    }

    bool openSerial() {
        closeSerial();
        int fd = ::open(port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
        if (fd < 0) {
            RCLCPP_ERROR_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                2000,
                "open %s failed: %s",
                port_.c_str(),
                std::strerror(errno)
            );
            return false;
        }

        struct termios tio;
        if (tcgetattr(fd, &tio) != 0) {
            RCLCPP_ERROR(this->get_logger(), "tcgetattr failed: %s", std::strerror(errno));
            ::close(fd);
            return false;
        }

        // 配置 8N1，无流控，原始模式
        cfmakeraw(&tio);
        tio.c_cflag |= (CLOCAL | CREAD);
        tio.c_cflag &= ~CRTSCTS;
        tio.c_cflag &= ~CSTOPB;
        tio.c_cflag &= ~PARENB;
        tio.c_cflag &= ~CSIZE;
        tio.c_cflag |= CS8;

        speed_t spd = to_baud_constant(baud_);
        cfsetispeed(&tio, spd);
        cfsetospeed(&tio, spd);

        tio.c_cc[VMIN] = 1;
        tio.c_cc[VTIME] = 1;

        if (tcsetattr(fd, TCSANOW, &tio) != 0) {
            RCLCPP_ERROR(this->get_logger(), "tcsetattr failed: %s", std::strerror(errno));
            ::close(fd);
            return false;
        }

        int flags = fcntl(fd, F_GETFL, 0);
        if (flags >= 0) {
            fcntl(fd, F_SETFL, flags & ~O_NONBLOCK);
        }

        {
            std::lock_guard<std::mutex> lock(fd_mutex_);
            fd_ = fd;
        }
        RCLCPP_INFO(this->get_logger(), "Serial opened");
        return true;
    }

    void closeSerial() {
        int fd_to_close = -1;
        {
            std::lock_guard<std::mutex> lock(fd_mutex_);
            if (fd_ >= 0) {
                fd_to_close = fd_;
                fd_ = -1;
            }
        }
        if (fd_to_close >= 0) {
            ::close(fd_to_close);
            RCLCPP_INFO(this->get_logger(), "Serial closed");
        }
    }

    // members
    std::string port_;
    int baud_ { 115200 };
    int reopen_interval_ms_ { 500 };
    bool rx_diagnostics_ { true };
    int nav_rx_log_period_ms_ { 1000 };
    int aim_rx_log_period_ms_ { 5000 };
    std::chrono::milliseconds idle_sleep_duration_ { 1 };

    rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr tx_sub_;
    rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr rx_pub_;
    rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr aim_tx_sub_;
    rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr aim_rx_pub_;

    std::thread read_thread_;
    std::atomic<bool> running_ { false };
    mutable std::mutex fd_mutex_;
    std::mutex write_mutex_;
    int fd_ { -1 };
    std::vector<uint8_t> rx_buffer_;

    int currentFd() const {
        std::lock_guard<std::mutex> lock(fd_mutex_);
        return fd_;
    }

    static std::string formatBytes(const uint8_t* data, size_t size, size_t max_count) {
        static constexpr char kHex[] = "0123456789ABCDEF";
        const size_t count = std::min(size, max_count);
        std::string out;
        out.reserve(count * 5 + 8);
        for (size_t i = 0; i < count; ++i) {
            if (i > 0) {
                out += ' ';
            }
            const uint8_t b = data[i];
            out += "0x";
            out += kHex[(b >> 4) & 0x0F];
            out += kHex[b & 0x0F];
        }
        if (size > count) {
            out += " ...";
        }
        return out;
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SerialRwNode>());
    rclcpp::shutdown();
    return 0;
}
