#!/usr/bin/env python3
import os, csv, math, time, bisect
from typing import Dict, List, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory

class RMSFallbackLogger(Node):
    """
    Fallback RMS logger: compares /joint_states (actual) to the most recent
    command on /<controller>/joint_trajectory (desired), by linearly
    interpolating the trajectory points over time since receipt.

    Params:
      controller_name:   'joint_trajectory_controller'
      record_secs:       how long to log (s)
      out_csv:           where to save CSV
      joints:            optional list[str] filter; default -> all in trajectory
    """
    def __init__(self):
        super().__init__('rms_tracking_fallback')

        self.declare_parameter('controller_name', 'joint_trajectory_controller')
        self.declare_parameter('record_secs', 8.0)
        self.declare_parameter('out_csv', 'rms_tracking_fallback.csv')
        self.declare_parameter('joints', [])

        self.controller = self.get_parameter('controller_name').get_parameter_value().string_value
        self.record_secs = float(self.get_parameter('record_secs').get_parameter_value().double_value)
        self.out_csv = os.path.expanduser(self.get_parameter('out_csv').get_parameter_value().string_value)
        self.joints_param = list(self.get_parameter('joints').get_parameter_value().string_array_value)

        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST, depth=100)

        # Subscriptions: commanded trajectory + actual states
        self.create_subscription(JointTrajectory,
                                 f'/{self.controller}/joint_trajectory',
                                 self._traj_cb, qos)
        self.create_subscription(JointState,
                                 '/joint_states',
                                 self._js_cb, qos)

        # Trajectory storage
        self.traj_joint_order: List[str] = []
        self.traj_times: List[float] = []   # seconds from start
        self.traj_pts: List[List[float]] = []  # positions in traj_joint_order
        self.traj_t0_monotonic: Optional[float] = None  # when we received first traj msg

        # Logging
        self.rows: List[List[float]] = []
        self.sum_sq: List[float] = []
        self.count: int = 0
        self.keep_indices: List[int] = []   # indices in traj_joint_order we track
        self.kept_names: List[str] = []

        # Start/stop
        self.t_start = time.monotonic()
        self.create_timer(self.record_secs, self._finish)

        self.get_logger().info(
            f"[Fallback] Recording RMS for {self.record_secs:.1f}s; "
            f"waiting for /{self.controller}/joint_trajectory and /joint_states"
        )

    # ---- Callbacks ----
    def _traj_cb(self, msg: JointTrajectory):
        # On new trajectory, reset timing and cache points
        if not msg.joint_names or not msg.points:
            return

        self.traj_joint_order = list(msg.joint_names)
        self.traj_times = [p.time_from_start.sec + p.time_from_start.nanosec * 1e-9 for p in msg.points]
        self.traj_pts = [list(p.positions) for p in msg.points]
        self.traj_t0_monotonic = time.monotonic()

        # pick indices to keep (once)
        if not self.keep_indices:
            if self.joints_param:
                self.keep_indices = [self.traj_joint_order.index(j) for j in self.joints_param
                                     if j in self.traj_joint_order]
            else:
                self.keep_indices = list(range(len(self.traj_joint_order)))
            self.kept_names = [self.traj_joint_order[i] for i in self.keep_indices]
            self.sum_sq = [0.0] * len(self.keep_indices)

        self.get_logger().info(
            f"Captured trajectory with {len(self.traj_pts)} points for joints: {self.traj_joint_order}"
        )

    def _js_cb(self, msg: JointState):
        # Need a trajectory first
        if self.traj_t0_monotonic is None:
            return

        # Build actual vector aligned to traj joint order
        name_to_pos = {n: p for n, p in zip(msg.name, msg.position)}
        actual = [name_to_pos.get(j, 0.0) for j in self.traj_joint_order]

        # Current elapsed time since trajectory start
        t_now = time.monotonic() - self.traj_t0_monotonic
        if t_now < 0.0:
            return

        # Interpolate desired at t_now
        desired = self._interp_desired(t_now)

        # Keep only selected joints
        row = [t_now]
        for out_idx, j_idx in enumerate(self.keep_indices):
            des = desired[j_idx]
            act = actual[j_idx]
            err = act - des
            self.sum_sq[out_idx] += err * err
            row.extend([des, act, err])

        self.rows.append(row)
        self.count += 1

    # ---- Helpers ----
    def _interp_desired(self, t: float) -> List[float]:
        # Before first point -> use first, after last -> use last
        if not self.traj_times:
            return [0.0] * 0
        if t <= self.traj_times[0]:
            return self.traj_pts[0]
        if t >= self.traj_times[-1]:
            return self.traj_pts[-1]

        # Find segment
        i = bisect.bisect_right(self.traj_times, t)
        i0 = i - 1
        i1 = i
        t0, t1 = self.traj_times[i0], self.traj_times[i1]
        if t1 <= t0:
            alpha = 0.0
        else:
            alpha = (t - t0) / (t1 - t0)

        p0 = self.traj_pts[i0]
        p1 = self.traj_pts[i1]
        # Linear interpolation per joint
        return [p0[k] + alpha * (p1[k] - p0[k]) for k in range(len(p0))]

    # ---- Finish ----
    def _finish(self):
        os.makedirs(os.path.dirname(self.out_csv) or ".", exist_ok=True)

        # header
        header = ['time_s']
        for name in self.kept_names:
            header += [f'{name}_desired', f'{name}_actual', f'{name}_error']

        with open(self.out_csv, 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(header)
            w.writerows(self.rows)

        if self.count > 0 and self.sum_sq:
            rms = [math.sqrt(s / self.count) for s in self.sum_sq]
            pretty = ', '.join(f'{n}: {v:.4f} rad' for n, v in zip(self.kept_names, rms))
            overall_mean = sum(rms) / len(rms)
            overall_max = max(rms)
            self.get_logger().info(f"Samples: {self.count}")
            self.get_logger().info(f"RMS per joint -> {pretty}")
            self.get_logger().info(f"RMS overall (mean): {overall_mean:.4f} rad; (max): {overall_max:.4f} rad")
        else:
            self.get_logger().warn("No trajectory and/or joint states captured; RMS unavailable.")

        self.get_logger().info(f"Saved CSV: {os.path.abspath(self.out_csv)}")
        self.destroy_node()
        rclpy.shutdown()

def main():
    rclpy.init()
    node = RMSFallbackLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()

