// #include "ros/ros.h"
#include <cstdint>
#include <map>
#include <string>

#pragma pack(1)
typedef struct { // 电控发送的命令数据
    uint8_t frame_header; // 8位 帧头 0x72
    uint8_t color; // 8位 机器人颜色（0=RED, 1=BLUE）
    // uint8_t  sentry_command;  // 8位 命令
    uint8_t eSentryState; // 8位 当前状态
    // uint8_t  eSentryEvent;    // 8位 事件
    uint16_t hp_remain; // 16位 剩余生命值
    uint16_t bullet_remain; // 16位 剩余子弹值
    float target_x; // 32位 目标x坐标 (odom坐标系，当需要停止时发送x_current)
    float target_y; // 32位 目标y坐标 (odom坐标系，当需要停止时发送y_current)
    // float    time_remain;     // 32位 剩余时间，单位秒
    uint8_t is_revive;
    float enemy_x; // 32位 敌方x坐标 (odom坐标系)
    float enemy_y; // 32位 敌方y坐标 (odom坐标系)
    uint8_t patrol_region; // 8位 巡逻区域，patrol_region_t
    uint8_t motion_allowed; // 8位 是否允许移动，非死亡状态/非比赛结束状态（0=不允许，1=允许）
    uint32_t reserve_1 : 8; // 16位 保留
    uint32_t reserve_2 : 32; // 32位 保留
    uint32_t reserve_3 : 32; // 32位 保留
    uint32_t reserve_4 : 32; // 32位 保留
    uint32_t reserve_5 : 32; // 32位 保留
    uint32_t reserve_6 : 32; // 32位 保留
    uint32_t reserve_7 : 32; // 32位 保留
    uint32_t reserve_8 : 32; // 32位 保留
    uint32_t reserve_9 : 32; // 32位 保留
    uint32_t reserve_10 : 32; // 32位 保留
    uint8_t frame_tail; // 帧尾 0x21
} navCommand_t;
#pragma pack()

#pragma pack(1)
typedef struct { // 发给电控：速度用机器人坐标系，位置用 odom，yaw 用 map
    uint8_t frame_header; // 8位帧头 0x72
    float x_speed; // base_link x 方向速度，前为正
    float y_speed; // base_link y 方向速度，左为正
    float x_current; // odom 当前 x 坐标
    float y_current; // odom 当前 y 坐标
    float x_target; // odom 目标 x 坐标
    float y_target; // odom 目标 y 坐标
    float yaw_current; // map 下当前 base_link 偏航角
    float yaw_desired; // map 下期望 base_link 偏航角
    uint8_t sentry_region; // 8位哨兵区域
    float time_test; // 32位测试时间，单位秒//没用
    uint8_t nav_state; // 状态
    uint8_t point_id; // 巡逻区号
    uint8_t target_region; //敌方车所在的区域
    uint8_t self_region; //自身所在的区域
    uint8_t chises_trapped; // 8位 1=fluctuate1/3 与 fluctuate2/4 均无法通过
    uint8_t narrow_passage; // 8位窄道标志：1=检测到窄道，0=普通通道
    uint32_t reserve_4 : 24; // 24位保留
    uint32_t reserve_5 : 32; // 32位保留
    uint32_t reserve_6 : 32; // 32位保留
    uint32_t reserve_7 : 32; // 32位保留
    uint32_t reserve_8 : 32; // 32位保留
    uint8_t frame_tail; // 帧尾 0x4D,填充到 64字节，保持帧尾为最后一字节
} navInfo_t;
#pragma pack()

#if defined(__cplusplus)
static_assert(sizeof(navInfo_t) == 64, "navInfo_t must be 64 bytes");
static_assert(sizeof(navCommand_t) == 64, "navCommand_t must be 64 bytes");
#endif

enum sentry_region {
    local = 0,
    flat,
    hole,
    slope,
    step,
    fluctuate,
};

typedef enum sentry_state {
    standby = 0, // 待命状态
    attack, // 进攻状态，该状态需视野中出现敌人
    patrol, // 巡逻状态，固定几个点的巡逻状态
    stationary_defense, // 原地不动防守状态，用于导航异常
    constrained_defense, // 约束防守状态，上堡垒
    error, // 错误状态，即进入了不该进入的状态转移表
    logic, // 逻辑状态，在这里面做逻辑处理和强制状态转换
    pursuit, // 追击状态，此时追击坐标为敌人消失的坐标
    supply, // 补给状态，弹丸打完或血量低下会进入此状态
    go_attack_outpost, // 只推前哨站状态
    hit_energy_buff, // 打符
    rush_base, // 冲基地
    occupy_point, // 占点,RMUL专供
    repel, // 驱赶，RMUL专供
} sentry_state_e;

typedef enum {
    opposite_half_, // 对方半场
    self_half_, // 我方半场
    self_fort_, // 我方堡垒
    opposite_fort_, // 对方堡垒
    self_highway_area_, // 我方高速区
    opposite_highway_area_, // 对方高速区
    central_highland_area_, // 中央高地
    rebuild_outpost_, // 重建前哨站
    opposite_base_area_, // 对方基地
} patrol_region_t;

enum sentry_event_e {
    none = 0,
    match_begin, // 比赛开始
    turn_off_auto, // 关自动模式
    turn_on_auto, // 开自动模式
    target_detected, // 目标出现
    target_disappear, // 目标消失
    being_hit, // 受击
    ammunition_capacity_reached_limit, // 发弹量达到上限
    HP_low, // 生命值低下
    logic_operate, // 逻辑操作，作为debug判断，不会有实际用处
    HP_regeneration, // 生命值恢复
    revive, // 复活
    outpost_destroyed, // 我方前哨站被击毁
    radar_lock_target, // 雷达锁定目标
    operator_intervene, // 云台手干预
};

enum target_region_e {
    self_base_area = 0,
    central_highland_area,
    enemy_base_area,
};

static const std::map<uint8_t, std::string> state_map = {
    { sentry_state_e::standby, "standby" },
    { sentry_state_e::attack, "attack" },
    { sentry_state_e::patrol, "patrol" },
    { sentry_state_e::stationary_defense, "stationary_defense" },
    { sentry_state_e::constrained_defense, "constrained_defense" },
    { sentry_state_e::error, "error" },
    { sentry_state_e::logic, "logic" },
    { sentry_state_e::pursuit, "pursuit" },
    { sentry_state_e::supply, "supply" },
    { sentry_state_e::go_attack_outpost, "go_attack_outpost" },
    { sentry_state_e::hit_energy_buff, "hit_energy_buff" },
    { sentry_state_e::rush_base, "rush_base" },
    { sentry_state_e::occupy_point, "occupy_point" },
    { sentry_state_e::repel, "repel" }
};

static const std::map<uint8_t, std::string> event_map = {
    { sentry_event_e::none, "none" },
    { sentry_event_e::match_begin, "match_begin" },
    { sentry_event_e::turn_off_auto, "turn_off_auto" },
    { sentry_event_e::turn_on_auto, "turn_on_auto" },
    { sentry_event_e::target_detected, "target_detected" },
    { sentry_event_e::target_disappear, "target_disappear" },
    { sentry_event_e::being_hit, "being_hit" },
    { sentry_event_e::ammunition_capacity_reached_limit, "ammunition_capacity_reached_limit" },
    { sentry_event_e::HP_low, "HP_low" },
    { sentry_event_e::logic_operate, "logic_operate" },
    { sentry_event_e::HP_regeneration, "HP_regeneration" },
    { sentry_event_e::revive, "revive" },
    { sentry_event_e::outpost_destroyed, "outpost_destroyed" },
    { sentry_event_e::radar_lock_target, "radar_lock_target" },
    { sentry_event_e::operator_intervene, "operator_intervene" }
};
