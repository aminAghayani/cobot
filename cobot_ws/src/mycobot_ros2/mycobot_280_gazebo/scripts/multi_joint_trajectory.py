#!/usr/bin/env python3
import sys, time
from typing import Dict, List

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.srv import GetParameters
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import JointTrajectoryControllerState

# ======= EDIT ME (students tweak only this block) =======
DURATION_PER_SEG = 2.0      # seconds per waypoint segment
FINAL_HOLD       = 1.0      # seconds to hold last pose
USE_CURRENT_START = True     # start from current /joint_states

WAYPOINTS: List[Dict[str, float]] = [
    {"joint2_to_joint1": 0.30, "joint3_to_joint2": 0.25},
    {"joint4_to_joint3": 0.40, "joint5_to_joint4": -0.30},
    {"joint6_to_joint5": 0.20, "joint6output_to_joint6": -0.50},
]
# ---- settle criteria & timeout ----
GOAL_TOL       = 0.01   # rad (max position error)
SETTLE_VEL     = 0.02   # rad/s (max velocity)
EXTRA_WAIT_SEC = 3.0    # extra wait after final time before giving up
# ========================================================

class MultiJointTrajSender(Node):
    def __init__(self):
        super().__init__("multi_joint_traj_sender")
        self.declare_parameter("controller_name", "joint_trajectory_controller")
        self.controller = self.get_parameter("controller_name").get_parameter_value().string_value

        # 1) get controller joint order
        cli = self.create_client(GetParameters, f"/{self.controller}/get_parameters")
        if not self._wait(cli, 10.0):
            self.get_logger().error("controller param service not available")
            rclpy.shutdown(); sys.exit(1)
        req = GetParameters.Request(); req.names = ["joints"]
        fut = cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
        vals = fut.result().values if fut.result() else []
        if not vals or vals[0].type != ParameterType.PARAMETER_STRING_ARRAY:
            self.get_logger().error("failed to read controller 'joints'")
            rclpy.shutdown(); sys.exit(1)
        self.joint_order = list(vals[0].string_array_value)
        self.get_logger().info(f"Controller joints: {self.joint_order}")

        # 2) seed from /joint_states
        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=50)
        self.latest: Dict[str, float] = {}
        self.create_subscription(JointState, "/joint_states", self._js_cb, qos)
        if USE_CURRENT_START:
            t0 = time.time()
            while rclpy.ok() and not self.latest and (time.time() - t0) < 3.0:
                rclpy.spin_once(self, timeout_sec=0.1)

        # 3) build trajectory points
        pts: List[JointTrajectoryPoint] = []
        current = [self.latest.get(j, 0.0) for j in self.joint_order] if USE_CURRENT_START else [0.0]*len(self.joint_order)
        tfs = 0.0

        # initial (t = 0)
        p0 = JointTrajectoryPoint()
        p0.positions = list(current)
        p0.time_from_start.sec = 0; p0.time_from_start.nanosec = 0
        pts.append(p0)

        for wp in WAYPOINTS:
            for j, v in wp.items():
                if j in self.joint_order:
                    current[self.joint_order.index(j)] = float(v)
                else:
                    self.get_logger().warn(f"Unknown joint in waypoint: {j}")
            tfs += DURATION_PER_SEG
            p = JointTrajectoryPoint()
            p.positions = list(current)
            p.time_from_start.sec = int(tfs)
            p.time_from_start.nanosec = int((tfs - int(tfs))*1e9)
            pts.append(p)

        if FINAL_HOLD > 0.0:
            tfs += FINAL_HOLD
            p_hold = JointTrajectoryPoint()
            p_hold.positions = list(current)
            p_hold.time_from_start.sec = int(tfs)
            p_hold.time_from_start.nanosec = int((tfs - int(tfs))*1e9)
            pts.append(p_hold)

        self.total_time = tfs

        # 4) publish once
        self.pub = self.create_publisher(JointTrajectory, f"/{self.controller}/joint_trajectory", qos)
        t0 = time.time()
        while rclpy.ok() and self.pub.get_subscription_count() == 0 and (time.time() - t0) < 2.0:
            rclpy.spin_once(self, timeout_sec=0.05)

        traj = JointTrajectory()
        traj.joint_names = self.joint_order
        traj.points = pts
        self.get_logger().info(f"Sending {len(pts)} points; total {self.total_time:.2f}s")
        self.pub.publish(traj)

        # 5) monitor controller state to auto-exit
        self.have_state = False
        self.last_err = 999.0
        self.last_vel = 999.0
        self.deadline = time.time() + self.total_time + EXTRA_WAIT_SEC

        self.create_subscription(JointTrajectoryControllerState,
                                 f"/{self.controller}/state",
                                 self._state_cb, qos)
        self.create_timer(0.05, self._check_done)

    # --- callbacks ---
    def _js_cb(self, msg: JointState):
        for n, p in zip(msg.name, msg.position):
            self.latest[n] = p

    def _state_cb(self, msg: JointTrajectoryControllerState):
        self.have_state = True
        try:
            if msg.desired.positions and msg.actual.positions:
                errs = [abs(a - d) for a, d in zip(msg.actual.positions, msg.desired.positions)]
                vels = [abs(v) for v in (msg.actual.velocities or [])] if msg.actual.velocities else [0.0]
                self.last_err = max(errs) if errs else 0.0
                self.last_vel = max(vels) if vels else 0.0
        except Exception:
            pass

    def _check_done(self):
        now = time.time()
        if self.have_state:
            if self.last_err <= GOAL_TOL and self.last_vel <= SETTLE_VEL:
                self.get_logger().info("Trajectory finished")
                self._shutdown()
                return
        if now > self.deadline:
            if not self.have_state:
                self.get_logger().warn("No controller state received; assuming sent. Exiting.")
            else:
                self.get_logger().warn("Trajectory wait timed out; exiting")
            self._shutdown()

    def _shutdown(self):
        self.destroy_node()
        rclpy.shutdown()

    def _wait(self, client, timeout):
        t0 = time.time()
        while rclpy.ok() and not client.wait_for_service(timeout_sec=0.2):
            if time.time() - t0 > timeout:
                return False
        return True

def main():
    rclpy.init()
    try:
        node = MultiJointTrajSender()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()

