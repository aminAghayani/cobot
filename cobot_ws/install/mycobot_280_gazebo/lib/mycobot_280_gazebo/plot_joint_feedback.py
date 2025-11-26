#!/usr/bin/env python3
import csv, os
from typing import List

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import JointState

class JointFeedbackRecorder(Node):
    def __init__(self):
        super().__init__('joint_feedback_recorder')

        # ----- Parameters -----
        self.declare_parameter('joint_name', '')
        self.declare_parameter('record_secs', 6.0)            # how long to record [s]
        self.declare_parameter('out_csv', 'joint_feedback.csv')
        self.declare_parameter('out_png', '')                 # empty => don't save PNG
        self.declare_parameter('show_plot', False)            # False => headless (no GUI)

        self.joint_name: str = self.get_parameter('joint_name').get_parameter_value().string_value
        self.record_secs: float = float(self.get_parameter('record_secs').get_parameter_value().double_value)
        self.out_csv: str = os.path.expanduser(self.get_parameter('out_csv').get_parameter_value().string_value)
        self.out_png: str = os.path.expanduser(self.get_parameter('out_png').get_parameter_value().string_value)
        self.show_plot: bool = bool(self.get_parameter('show_plot').get_parameter_value().bool_value)

        if not self.joint_name:
            self.get_logger().error("Set -p joint_name:=<your_joint>")
            rclpy.shutdown()
            return

        # ----- Subscribe to /joint_states -----
        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST, depth=50)
        self.create_subscription(JointState, '/joint_states', self._cb, qos)

        self.t0 = None
        self.t: List[float] = []
        self.pos: List[float] = []

        # Stop after record_secs
        self.create_timer(self.record_secs, self._finish)

        self.get_logger().info(
            f"Recording '{self.joint_name}' for {self.record_secs:.1f}s… (saving CSV to {self.out_csv})"
        )

    def _cb(self, msg: JointState):
        if self.t0 is None:
            self.t0 = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        try:
            idx = msg.name.index(self.joint_name)
        except ValueError:
            return  # joint not in this message
        now = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.t.append(now - self.t0)
        self.pos.append(msg.position[idx])

    def _finish(self):
        # ----- Save CSV -----
        os.makedirs(os.path.dirname(self.out_csv) or ".", exist_ok=True)
        with open(self.out_csv, 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['time_s', 'position_rad'])
            w.writerows(zip(self.t, self.pos))
        self.get_logger().info(f"CSV saved: {os.path.abspath(self.out_csv)} ({len(self.t)} samples)")

        # ----- Plot (headless unless show_plot=True) -----
        try:
            import matplotlib
            if not self.show_plot:
                matplotlib.use("Agg")  # no GUI backend
            import matplotlib.pyplot as plt

            plt.figure()
            plt.plot(self.t, self.pos, marker='.')
            plt.xlabel('Time [s]')
            plt.ylabel(f"{self.joint_name} position [rad]")
            plt.title(f"Feedback of {self.joint_name}")
            plt.grid(True)

            if self.out_png:
                os.makedirs(os.path.dirname(self.out_png) or ".", exist_ok=True)
                plt.savefig(self.out_png, dpi=150, bbox_inches='tight')
                self.get_logger().info(f"PNG saved: {os.path.abspath(self.out_png)}")

            if self.show_plot:
                plt.show()
            else:
                plt.close('all')
        except Exception as e:
            self.get_logger().warn(f"Plot skipped: {e}. You can plot the CSV later.")

        # Exit cleanly so launches don't hang
        self.destroy_node()
        rclpy.shutdown()

def main():
    rclpy.init()
    node = JointFeedbackRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()

