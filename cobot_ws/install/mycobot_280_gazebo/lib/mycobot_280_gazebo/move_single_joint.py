#!/usr/bin/env python3
import sys, time
from typing import Dict, List

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.srv import GetParameters
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import JointTrajectoryControllerState  # NEW
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class SingleJointMover(Node):
    def __init__(self):
        super().__init__('single_joint_mover')

        # Params
        self.declare_parameter('controller_name', 'joint_trajectory_controller')
        self.declare_parameter('joint_name', '')
        self.declare_parameter('position', 0.0)
        self.declare_parameter('duration', 2.0)
        self.declare_parameter('goal_tolerance', 0.01)   # NEW (rad)
        self.declare_parameter('settle_velocity', 0.02)  # NEW (rad/s)
        self.declare_parameter('max_wait', 10.0)         # NEW (s) after command send

        self.controller_name = self.get_parameter('controller_name').get_parameter_value().string_value
        self.target_joint   = self.get_parameter('joint_name').get_parameter_value().string_value
        self.target_pos     = self.get_parameter('position').get_parameter_value().double_value
        self.duration       = self.get_parameter('duration').get_parameter_value().double_value
        self.goal_tol       = float(self.get_parameter('goal_tolerance').get_parameter_value().double_value)
        self.settle_vel     = float(self.get_parameter('settle_velocity').get_parameter_value().double_value)
        self.max_wait       = float(self.get_parameter('max_wait').get_parameter_value().double_value)

        if not self.target_joint:
            self.get_logger().error("Parameter 'joint_name' is required.")
            rclpy.shutdown(); sys.exit(1)

        # Query controller joints via parameter service
        ctrl_node = f'/{self.controller_name}'
        cli = self.create_client(GetParameters, f'{ctrl_node}/get_parameters')
        if not self._wait_for_service(cli, 10.0):
            self.get_logger().error(f"Timed out waiting for {cli.srv_name}")
            rclpy.shutdown(); sys.exit(1)

        req = GetParameters.Request(); req.names = ['joints']
        fut = cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
        if not fut.result():
            self.get_logger().error("Failed to retrieve 'joints' parameter from controller.")
            rclpy.shutdown(); sys.exit(1)
        vals = fut.result().values
        if not vals or vals[0].type != ParameterType.PARAMETER_STRING_ARRAY:
            self.get_logger().error("Controller 'joints' parameter missing or wrong type.")
            rclpy.shutdown(); sys.exit(1)

        self.controller_joints: List[str] = list(vals[0].string_array_value)
        if self.target_joint not in self.controller_joints:
            self.get_logger().error(f"Requested joint '{self.target_joint}' not in {self.controller_joints}")
            rclpy.shutdown(); sys.exit(1)

        # Read /joint_states once (hold others)
        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=10)
        self.latest_positions: Dict[str, float] = {}
        self.received_js = False
        self.create_subscription(JointState, '/joint_states', self._js_cb, qos)

        self.get_logger().info("Waiting for /joint_states ...")
        t0 = time.time()
        while rclpy.ok() and not self.received_js and (time.time() - t0) < 5.0:
            rclpy.spin_once(self, timeout_sec=0.1)
        if not self.received_js:
            self.get_logger().warn("No /joint_states in 5s; using 0.0 for any missing joints.")

        # Build target for all joints
        positions = [self.target_pos if j == self.target_joint else self.latest_positions.get(j, 0.0)
                     for j in self.controller_joints]

        # Publish trajectory
        self.pub = self.create_publisher(JointTrajectory, f'/{self.controller_name}/joint_trajectory', qos)
        t_sub = time.time()
        while rclpy.ok() and self.pub.get_subscription_count() == 0 and (time.time() - t_sub) < 2.0:
            rclpy.spin_once(self, timeout_sec=0.05)

        traj = JointTrajectory(); traj.joint_names = self.controller_joints
        pt = JointTrajectoryPoint(); pt.positions = positions
        pt.time_from_start.sec = int(self.duration)
        pt.time_from_start.nanosec = int((self.duration - int(self.duration)) * 1e9)
        traj.points.append(pt)

        self.get_logger().info(
            f"Sending: {self.target_joint} -> {self.target_pos:.3f} rad in {self.duration:.1f}s (others held)"
        )
        self.pub.publish(traj)

        # NEW: monitor controller state until goal reached or timeout, then exit
        self._goal_deadline = time.time() + max(self.duration, 0.0) + self.max_wait
        self._joint_index = self.controller_joints.index(self.target_joint)
        state_topic = f'/{self.controller_name}/state'
        self._have_state = False
        self._last_err = None

        self.create_subscription(JointTrajectoryControllerState, state_topic, self._state_cb, qos)
        self.create_timer(0.05, self._check_goal)  # 20 Hz check

    def _js_cb(self, msg: JointState):
        for n, p in zip(msg.name, msg.position):
            self.latest_positions[n] = p
        self.received_js = True

    def _state_cb(self, msg: JointTrajectoryControllerState):  # NEW
        self._have_state = True
        try:
            pos = msg.actual.positions[self._joint_index]
            vel = msg.actual.velocities[self._joint_index] if msg.actual.velocities else 0.0
            self._last_err = abs(pos - self.target_pos), abs(vel)
        except Exception:
            pass

    def _check_goal(self):  # NEW
        now = time.time()
        if not self._have_state:
            if now > self._goal_deadline:
                self.get_logger().warn("No controller state received; assuming sent. Exiting.")
                self._shutdown()
            return

        if self._last_err is not None:
            pos_err, vel_abs = self._last_err
            if pos_err <= self.goal_tol and vel_abs <= self.settle_vel:
                self.get_logger().info("Goal reached")
                self._shutdown()
                return

        if now > self._goal_deadline:
            self.get_logger().warn("Goal wait timed out; exiting")
            self._shutdown()

    def _shutdown(self):
        self.destroy_node()
        rclpy.shutdown()

    def _wait_for_service(self, client, timeout):
        t0 = time.time()
        while rclpy.ok() and not client.wait_for_service(timeout_sec=0.2):
            if (time.time() - t0) > timeout:
                return False
        return True

def main():
    rclpy.init()
    try:
        node = SingleJointMover()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()

