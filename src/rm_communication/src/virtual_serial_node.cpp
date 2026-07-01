#include "protocol.h"

#include <algorithm>
#include <chrono>
#include <cstring>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <string>
#include <unordered_map>
#include <vector>

using namespace std::chrono_literals;

namespace {

constexpr uint8_t kNavHeader = 0x72;
constexpr uint8_t kNavTailRx = 0x21;
constexpr uint8_t kNavTailTx = 0x4D;
constexpr size_t kFrameSize = 64;

static_assert(sizeof(navCommand_t) == kFrameSize, "navCommand_t must be 64 bytes");
static_assert(sizeof(navInfo_t) == kFrameSize, "navInfo_t must be 64 bytes");

const std::unordered_map<std::string, uint8_t> kStateNameToValue = {
    { "standby", sentry_state_e::standby },
    { "attack", sentry_state_e::attack },
    { "patrol", sentry_state_e::patrol },
    { "stationary_defense", sentry_state_e::stationary_defense },
    { "constrained_defense", sentry_state_e::constrained_defense },
    { "constrained_defence", sentry_state_e::constrained_defense },
    { "error", sentry_state_e::error },
    { "logic", sentry_state_e::logic },
    { "pursuit", sentry_state_e::pursuit },
    { "supply", sentry_state_e::supply },
    { "go_attack_outpost", sentry_state_e::go_attack_outpost },
    { "hit_energy_buff", sentry_state_e::hit_energy_buff },
    { "rush_base", sentry_state_e::rush_base },
    { "occupy_point", sentry_state_e::occupy_point },
    { "repel", sentry_state_e::repel },
};

uint8_t clampToU8(int64_t value) {
    return static_cast<uint8_t>(std::clamp<int64_t>(value, 0, 255));
}

uint16_t clampToU16(int64_t value) {
    return static_cast<uint16_t>(std::clamp<int64_t>(value, 0, 65535));
}

} // namespace

class VirtualSerialNode: public rclcpp::Node {
public:
    VirtualSerialNode(): Node("virtual_serial_node") {
        this->declare_parameter<double>("publish_hz", 20.0);
        this->declare_parameter<int>("sim_rx_log_period_ms", 1000);
        this->declare_parameter<int>("sim_tx_log_period_ms", 1000);
        this->declare_parameter<int64_t>("color", 0);
        this->declare_parameter<int64_t>("state", sentry_state_e::standby);
        this->declare_parameter<int64_t>("patrol_region", self_half_);
        this->declare_parameter<int64_t>("hp", 600);
        this->declare_parameter<int64_t>("bullet", 400);
        this->declare_parameter<bool>("is_revive", false);
        this->declare_parameter<double>("target_x", 0.0);
        this->declare_parameter<double>("target_y", 0.0);
        this->declare_parameter<double>("enemy_x", 1.0);
        this->declare_parameter<double>("enemy_y", 0.0);
        this->declare_parameter<bool>("auto_sequence", false);
        this->declare_parameter<double>("sequence_period_s", 5.0);
        this->declare_parameter<std::vector<int64_t>>(
            "state_sequence",
            std::vector<int64_t> {
                sentry_state_e::standby,
                sentry_state_e::patrol,
                sentry_state_e::attack,
                sentry_state_e::pursuit,
                sentry_state_e::rush_base,
                sentry_state_e::supply,
            }
        );
        this->declare_parameter<std::vector<int64_t>>(
            "patrol_region_sequence",
            std::vector<int64_t> {
                self_half_,
                self_fort_,
                central_highland_area_,
                opposite_base_area_,
            }
        );

        rx_pub_ = this->create_publisher<std_msgs::msg::UInt8MultiArray>(
            "/rm_comm/rx_packet",
            rclcpp::QoS(10).reliable()
        );

        tx_sub_ = this->create_subscription<std_msgs::msg::UInt8MultiArray>(
            "/rm_comm/tx_packet",
            rclcpp::QoS(10).reliable(),
            std::bind(&VirtualSerialNode::onTxPacket, this, std::placeholders::_1)
        );

        state_sub_ = this->create_subscription<std_msgs::msg::UInt8>(
            "/sim_electrical/state",
            10,
            std::bind(&VirtualSerialNode::onState, this, std::placeholders::_1)
        );

        patrol_region_sub_ = this->create_subscription<std_msgs::msg::UInt8>(
            "/sim_electrical/patrol_region",
            10,
            std::bind(&VirtualSerialNode::onPatrolRegion, this, std::placeholders::_1)
        );

        state_name_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/sim_electrical/state_name",
            10,
            std::bind(&VirtualSerialNode::onStateName, this, std::placeholders::_1)
        );

        double hz = this->get_parameter("publish_hz").as_double();
        if (hz <= 0.0) {
            hz = 1.0;
        }
        sim_rx_log_period_ms_ = this->get_parameter("sim_rx_log_period_ms").as_int();
        sim_tx_log_period_ms_ = this->get_parameter("sim_tx_log_period_ms").as_int();
        publish_timer_ = this->create_wall_timer(
            std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::duration<double>(1.0 / hz)
            ),
            std::bind(&VirtualSerialNode::publishRxPacket, this)
        );

        sequence_start_ = this->now();
        RCLCPP_INFO(
            this->get_logger(),
            "virtual_serial_node started: publishing simulated navCommand_t on /rm_comm/rx_packet"
        );
    }

private:
    void onState(const std_msgs::msg::UInt8::SharedPtr msg) {
        state_override_ = msg->data;
        RCLCPP_INFO(this->get_logger(), "Sim state override: %u", msg->data);
    }

    void onPatrolRegion(const std_msgs::msg::UInt8::SharedPtr msg) {
        patrol_region_override_ = msg->data;
        RCLCPP_INFO(this->get_logger(), "Sim patrol_region override: %u", msg->data);
    }

    void onStateName(const std_msgs::msg::String::SharedPtr msg) {
        auto it = kStateNameToValue.find(msg->data);
        if (it == kStateNameToValue.end()) {
            RCLCPP_WARN(this->get_logger(), "Unknown state_name: %s", msg->data.c_str());
            return;
        }
        state_override_ = it->second;
        RCLCPP_INFO(this->get_logger(), "Sim state override: %s (%u)", msg->data.c_str(), it->second);
    }

    void publishRxPacket() {
        navCommand_t cmd {};
        cmd.frame_header = kNavHeader;
        cmd.color = clampToU8(this->get_parameter("color").as_int());
        cmd.eSentryState = currentState();
        cmd.hp_remain = clampToU16(this->get_parameter("hp").as_int());
        cmd.bullet_remain = clampToU16(this->get_parameter("bullet").as_int());
        cmd.target_x = static_cast<float>(this->get_parameter("target_x").as_double());
        cmd.target_y = static_cast<float>(this->get_parameter("target_y").as_double());
        cmd.is_revive = this->get_parameter("is_revive").as_bool() ? 1 : 0;
        cmd.enemy_x = static_cast<float>(this->get_parameter("enemy_x").as_double());
        cmd.enemy_y = static_cast<float>(this->get_parameter("enemy_y").as_double());
        cmd.patrol_region = currentPatrolRegion();
        cmd.frame_tail = kNavTailRx;

        std_msgs::msg::UInt8MultiArray msg;
        const uint8_t* bytes = reinterpret_cast<const uint8_t*>(&cmd);
        msg.data.assign(bytes, bytes + sizeof(navCommand_t));
        rx_pub_->publish(msg);

        RCLCPP_INFO_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            sim_rx_log_period_ms_,
            "SIM RX -> state=%u motion=%u patrol_region=%u hp=%u bullet=%u target=(%.2f, %.2f) enemy=(%.2f, %.2f)",
            cmd.eSentryState,
            cmd.motion_allowed,
            cmd.patrol_region,
            cmd.hp_remain,
            cmd.bullet_remain,
            cmd.target_x,
            cmd.target_y,
            cmd.enemy_x,
            cmd.enemy_y
        );
    }

    uint8_t currentState() {
        if (state_override_.has_value()) {
            return state_override_.value();
        }
        if (!this->get_parameter("auto_sequence").as_bool()) {
            return clampToU8(this->get_parameter("state").as_int());
        }
        auto sequence = this->get_parameter("state_sequence").as_integer_array();
        if (sequence.empty()) {
            return clampToU8(this->get_parameter("state").as_int());
        }
        const size_t index = sequenceIndex(sequence.size());
        return clampToU8(sequence[index]);
    }

    uint8_t currentPatrolRegion() {
        if (patrol_region_override_.has_value()) {
            return patrol_region_override_.value();
        }
        if (!this->get_parameter("auto_sequence").as_bool()) {
            return clampToU8(this->get_parameter("patrol_region").as_int());
        }
        auto sequence = this->get_parameter("patrol_region_sequence").as_integer_array();
        if (sequence.empty()) {
            return clampToU8(this->get_parameter("patrol_region").as_int());
        }
        const size_t index = sequenceIndex(sequence.size());
        return clampToU8(sequence[index]);
    }

    size_t sequenceIndex(size_t sequence_size) const {
        const double period = std::max(0.1, this->get_parameter("sequence_period_s").as_double());
        const double elapsed = (this->now() - sequence_start_).seconds();
        return static_cast<size_t>(elapsed / period) % sequence_size;
    }

    void onTxPacket(const std_msgs::msg::UInt8MultiArray::SharedPtr msg) {
        if (msg->data.size() != kFrameSize) {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                2000,
                "SIM TX packet size %zu != %zu",
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
                "SIM TX bad framing head=0x%02X tail=0x%02X",
                msg->data.front(),
                msg->data.back()
            );
            return;
        }

        navInfo_t info {};
        std::memcpy(&info, msg->data.data(), sizeof(navInfo_t));
        RCLCPP_INFO_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            sim_tx_log_period_ms_,
            "SIM TX <- speed=(%.2f, %.2f) current=(%.2f, %.2f) target=(%.2f, %.2f) nav_state=%u point_id=%u trapped=%u narrow=%u",
            info.x_speed,
            info.y_speed,
            info.x_current,
            info.y_current,
            info.x_target,
            info.y_target,
            info.nav_state,
            info.point_id,
            info.chises_trapped,
            info.narrow_passage
        );
    }

    rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr rx_pub_;
    rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr tx_sub_;
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr state_sub_;
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr patrol_region_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr state_name_sub_;
    rclcpp::TimerBase::SharedPtr publish_timer_;
    rclcpp::Time sequence_start_;
    std::optional<uint8_t> state_override_;
    std::optional<uint8_t> patrol_region_override_;
    int sim_rx_log_period_ms_ { 1000 };
    int sim_tx_log_period_ms_ { 1000 };
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VirtualSerialNode>());
    rclcpp::shutdown();
    return 0;
}
