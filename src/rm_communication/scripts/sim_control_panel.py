#!/usr/bin/env python3

import tkinter as tk
from tkinter import ttk

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, UInt8


STATES = [
    ("standby", "待命", 0),
    ("attack", "进攻/追击点", 1),
    ("patrol", "区域巡逻", 2),
    ("stationary_defense", "原地防守", 3),
    ("constrained_defense", "约束防守", 4),
    ("pursuit", "追击消失点", 7),
    ("supply", "补给", 8),
    ("go_attack_outpost", "推前哨站", 9),
    ("hit_energy_buff", "打符", 10),
    ("rush_base", "冲基地", 11),
    ("occupy_point", "占点", 12),
    ("repel", "驱赶", 13),
]

PATROL_REGIONS = [
    ("opposite_half", "对方半场", 0),
    ("self_half", "我方半场", 1),
    ("self_fort", "我方堡垒", 2),
    ("opposite_fort", "对方堡垒", 3),
    ("self_highway_area", "我方高速区", 4),
    ("opposite_highway_area", "对方高速区", 5),
    ("central_highland_area", "中央高地", 6),
    ("rebuild_outpost", "重建前哨站", 7),
    ("opposite_base_area", "对方基地", 8),
]


class SimControlNode(Node):
    def __init__(self):
        super().__init__("sim_control_panel")
        self.state_pub = self.create_publisher(String, "/sim_electrical/state_name", 10)
        self.region_pub = self.create_publisher(UInt8, "/sim_electrical/patrol_region", 10)

    def publish_state(self, state_name: str):
        msg = String()
        msg.data = state_name
        self.state_pub.publish(msg)
        self.get_logger().info(f"Set simulated state: {state_name}")

    def publish_region(self, region: int):
        msg = UInt8()
        msg.data = int(region)
        self.region_pub.publish(msg)
        self.get_logger().info(f"Set simulated patrol_region: {region}")


class SimControlPanel:
    def __init__(self, node: SimControlNode):
        self.node = node
        self.root = tk.Tk()
        self.root.title("RMUC Virtual Electrical Control")
        self.root.geometry("760x480")
        self.root.minsize(680, 420)

        self.current_state = tk.StringVar(value="standby")
        self.current_region = tk.StringVar(value="self_half")
        self.status = tk.StringVar(value="已连接虚拟电控话题")

        self._build()
        self.root.protocol("WM_DELETE_WINDOW", self.close)

    def _build(self):
        main = ttk.Frame(self.root, padding=12)
        main.pack(fill=tk.BOTH, expand=True)

        header = ttk.Frame(main)
        header.pack(fill=tk.X)
        ttk.Label(header, text="RMUC 仿真电控面板", font=("Sans", 16, "bold")).pack(side=tk.LEFT)
        ttk.Label(header, textvariable=self.status).pack(side=tk.RIGHT)

        body = ttk.PanedWindow(main, orient=tk.HORIZONTAL)
        body.pack(fill=tk.BOTH, expand=True, pady=(12, 0))

        state_frame = ttk.LabelFrame(body, text="状态切换", padding=10)
        body.add(state_frame, weight=3)
        self._add_state_buttons(state_frame)

        region_frame = ttk.LabelFrame(body, text="巡逻区域", padding=10)
        body.add(region_frame, weight=2)
        self._add_region_buttons(region_frame)

        footer = ttk.Frame(main)
        footer.pack(fill=tk.X, pady=(10, 0))
        ttk.Label(footer, text="当前状态:").pack(side=tk.LEFT)
        ttk.Label(footer, textvariable=self.current_state, width=22).pack(side=tk.LEFT, padx=(4, 16))
        ttk.Label(footer, text="当前巡逻区:").pack(side=tk.LEFT)
        ttk.Label(footer, textvariable=self.current_region, width=24).pack(side=tk.LEFT, padx=(4, 16))

    def _add_state_buttons(self, parent):
        for index, (name, label, value) in enumerate(STATES):
            button = ttk.Button(
                parent,
                text=f"{value:02d}  {label}",
                command=lambda n=name: self.set_state(n),
            )
            button.grid(row=index // 2, column=index % 2, sticky="ew", padx=4, pady=4)
        parent.columnconfigure(0, weight=1)
        parent.columnconfigure(1, weight=1)

    def _add_region_buttons(self, parent):
        for index, (name, label, value) in enumerate(PATROL_REGIONS):
            button = ttk.Button(
                parent,
                text=f"{value}  {label}",
                command=lambda n=name, v=value: self.set_region(n, v),
            )
            button.grid(row=index, column=0, sticky="ew", padx=4, pady=3)
        parent.columnconfigure(0, weight=1)

    def set_state(self, state_name: str):
        self.node.publish_state(state_name)
        self.current_state.set(state_name)
        self.status.set(f"已发送状态: {state_name}")

    def set_region(self, region_name: str, region_value: int):
        self.node.publish_region(region_value)
        self.current_region.set(region_name)
        self.status.set(f"已发送巡逻区: {region_name} ({region_value})")

    def spin_once(self):
        rclpy.spin_once(self.node, timeout_sec=0.0)
        self.root.after(20, self.spin_once)

    def run(self):
        self.root.after(20, self.spin_once)
        self.root.mainloop()

    def close(self):
        self.root.quit()


def main():
    rclpy.init()
    node = SimControlNode()
    try:
        panel = SimControlPanel(node)
        panel.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
