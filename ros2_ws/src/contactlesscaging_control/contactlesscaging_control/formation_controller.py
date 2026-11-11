"""A ROS 2 local PID controller for one robot in the four-robot formation.

Input poses MUST be in the same calibrated world frame, in metres, with ROS quaternions.
Output publishes Twist to the robot's namespace (normally cmd_vel).
"""
from __future__ import annotations

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseStamped, Twist
from .control_math import PID2D, heading_from_quaternion, planar_to_cmd

class FormationController(Node):
    def __init__(self):
        super().__init__('formation_controller')
        self.declare_parameter('robot_name', 'robot1')
        self.declare_parameter('leader_name', 'robot1')
        self.declare_parameter('role', 'leader')
        self.declare_parameter('offset_x', 0.0)
        self.declare_parameter('offset_y', 0.0)
        self.declare_parameter('kp', 0.9)
        self.declare_parameter('ki', 0.0)
        self.declare_parameter('kd', 0.05)
        self.declare_parameter('integral_limit', 0.2)
        self.declare_parameter('vmax', 0.05)
        self.declare_parameter('omega_max', 0.6)
        self.declare_parameter('heading_gain', 1.5)
        self.declare_parameter('control_rate_hz', 20.0)
        self.declare_parameter('pose_timeout_s', 0.5)
        self.declare_parameter('frame_id', 'world')
        self.declare_parameter('require_target', True)
        self.declare_parameter('enabled', False)
        p = lambda name: self.get_parameter(name).value
        self.robot_name = str(p('robot_name'))
        self.leader_name = str(p('leader_name'))
        self.backward = p('role') == 'leader'
        self.offset_x, self.offset_y = float(p('offset_x')), float(p('offset_y'))
        self.pid = PID2D(float(p('kp')),float(p('ki')),float(p('kd')),
                         integral_limit=float(p('integral_limit')))
        self.vmax = float(p('vmax'))
        self.omega_max = float(p('omega_max'))
        self.heading_gain = float(p('heading_gain'))
        self.pose_timeout = float(p('pose_timeout_s'))
        self.frame = str(p('frame_id'))
        self.require_target = bool(p('require_target'))
        self.local_pose = self.leader_pose = self.target_pose = None
        self.stamps = {}
        self.last_control_s = None
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.VOLATILE)
        self.create_subscription(PoseStamped, f'/phasespace/{self.robot_name}/pose',
                                 lambda msg: self._remember('local', msg), qos)
        if not self.backward:
            self.create_subscription(PoseStamped, f'/phasespace/{self.leader_name}/pose',
                                     lambda msg: self._remember('leader', msg), qos)
        else:
            self.create_subscription(PoseStamped, '/formation/leader_target',
                                     lambda msg: self._remember('target', msg), qos)
        self.publisher = self.create_publisher(Twist, 'cmd_vel', qos)
        self.create_timer(1.0 / max(1.0,float(p('control_rate_hz'))), self._step)
        self.get_logger().info(f'{self.robot_name}: {"backward leader" if self.backward else "forward follower"} controller ready')

    def _remember(self, kind, msg):
        if msg.header.frame_id and msg.header.frame_id != self.frame:
            self.get_logger().warn(f'Ignored {kind} pose in frame {msg.header.frame_id!r}; require {self.frame!r}', throttle_duration_sec=5.0)
            return
        setattr(self, {'local':'local_pose','leader':'leader_pose','target':'target_pose'}[kind], msg)
        self.stamps[kind] = self.get_clock().now().nanoseconds * 1e-9

    def _fresh(self, kind, now):
        return kind in self.stamps and 0 <= now - self.stamps[kind] <= self.pose_timeout

    def _stop(self):
        self.publisher.publish(Twist())
        self.pid.reset()

    def _step(self):
        if not self.get_parameter('enabled').value:
            self._stop();return
        now = self.get_clock().now().nanoseconds * 1e-9
        if not self._fresh('local',now):
            self._stop(); self.last_control_s = now; return
        if self.backward:
            if self.require_target and not self._fresh('target',now):
                self._stop(); self.last_control_s = now; return
            if not self.require_target and self.target_pose is None:
                self._stop(); self.last_control_s = now; return
            desired_x, desired_y = (self.target_pose.pose.position.x,
                                    self.target_pose.pose.position.y)
        else:
            if not self._fresh('leader',now):
                self._stop(); self.last_control_s = now; return
            desired_x = self.leader_pose.pose.position.x + self.offset_x
            desired_y = self.leader_pose.pose.position.y + self.offset_y
        if self.last_control_s is None:
            self.last_control_s = now; self._stop(); return
        dt = now - self.last_control_s
        self.last_control_s = now
        pos = self.local_pose.pose.position
        q = self.local_pose.pose.orientation
        qx, qy = self.pid.update(desired_x-pos.x, desired_y-pos.y, dt)
        yaw = heading_from_quaternion(q.x,q.y,q.z,q.w)
        vx, wz = planar_to_cmd(qx,qy,yaw, backward=self.backward,
                                vmax=self.vmax,omega_max=self.omega_max,
                                heading_gain=self.heading_gain)
        cmd = Twist(); cmd.linear.x = vx; cmd.angular.z = wz
        self.publisher.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = FormationController()
    try:
        rclpy.spin(node)
    finally:
        node._stop(); node.destroy_node(); rclpy.shutdown()

if __name__ == '__main__':
    main()
