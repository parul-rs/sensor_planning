#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
from gazebo_msgs.srv import SetEntityState
from gazebo_msgs.msg import EntityState, ModelStates
from geometry_msgs.msg import Pose, Twist, Vector3, Quaternion, Point

class ChaserHCWDynamics(Node):
    def __init__(self):
        super().__init__('chaser_hcw_dynamics')

        self.client = self.create_client(SetEntityState, '/set_entity_state')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /set_entity_state...')
        self.get_logger().info('/set_entity_state READY')

        self.target_name = 'target'
        self.chaser_name = 'chaser'
        self.target_pose = None

        self.model_sub = self.create_subscription(
            ModelStates, '/gazebo/model_states', self.model_cb, 10)

        self.declare_parameter('n', 0.2)
        self.declare_parameter('dt', 0.02)
        self.declare_parameter('x0', 0.5)
        self.declare_parameter('y0', 0.0)
        self.declare_parameter('z0', 0.0)
        self.declare_parameter('xdot0', 0.0)
        self.declare_parameter('ydot0', 0.0)
        self.declare_parameter('zdot0', 0.0)

        self.n = float(self.get_parameter('n').value)
        self.dt = float(self.get_parameter('dt').value)
        self.t = 0.0

        self.x0 = float(self.get_parameter('x0').value)
        self.y0 = float(self.get_parameter('y0').value)
        self.z0 = float(self.get_parameter('z0').value)
        self.xdot0 = float(self.get_parameter('xdot0').value)
        self.ydot0 = float(self.get_parameter('ydot0').value)
        self.zdot0 = float(self.get_parameter('zdot0').value)

        self.state = np.array([
            self.get_parameter('x0').value,
            self.get_parameter('y0').value,
            self.get_parameter('z0').value,
            self.get_parameter('xdot0').value,
            self.get_parameter('ydot0').value,
            self.get_parameter('zdot0').value
        ], dtype=float)

        self.timer = self.create_timer(self.dt, self.step)

    def model_cb(self, msg: ModelStates):
        try:
            idx = msg.name.index(self.target_name)
            self.target_pose = msg.pose[idx]
        except ValueError:
            return

    def hcw_dynamics(self, s):
        x, y, z, xd, yd, zd = s
        n = self.n
        return np.array([
            xd,
            yd,
            zd,
            3*n**2*x + 2*n*yd,
            -2*n*xd,
            -n**2*z
        ])

    def rk4(self, s, dt):
        k1 = self.hcw_dynamics(s)
        k2 = self.hcw_dynamics(s + 0.5*dt*k1)
        k3 = self.hcw_dynamics(s + 0.5*dt*k2)
        k4 = self.hcw_dynamics(s + dt*k3)
        return s + (dt/6)*(k1 + 2*k2 + 2*k3 + k4)

    def step(self):
        self.t += self.dt

        # Clohessy-Wiltshire relative motion
        x = (4 - 3 * np.cos(self.n * self.t)) * self.x0 + \
            (1 / self.n) * np.sin(self.n * self.t) * self.xdot0 + \
            (2 / self.n) * (1 - np.cos(self.n * self.t)) * self.ydot0

        y = 6 * (np.sin(self.n * self.t) - self.n * self.t) * self.x0 + \
            self.y0 - (2 / self.n) * (1 - np.cos(self.n * self.t)) * self.xdot0 + \
            (1 / self.n) * (4 * np.sin(self.n * self.t) - 3 * self.n * self.t) * self.ydot0

        z = self.z0 * np.cos(self.n * self.t) + (self.zdot0 / self.n) * np.sin(self.n * self.t)

        # Convert relative to absolute position around origin (assume target centered)
        pose = Pose()
        pose.position = Point(x=x, y=y, z=z)
        pose.orientation = Quaternion(w=1.0)

        twist = Twist()
        twist.linear = Vector3(x=0.0, y=0.0, z=0.0)

        state = EntityState()
        state.name = self.chaser_name
        state.pose = pose
        state.twist = twist
        state.reference_frame = 'world'

        req = SetEntityState.Request()
        req.state = state

        # for debugging: print position and service call status
        self.get_logger().info(f"[{self.chaser_name}] t={self.t:.2f} pos=({x:.2f}, {y:.2f}, {z:.2f})")
        future = self.client.call_async(req)

        def callback(fut):
            if fut.result() is not None:
                self.get_logger().info(f"[{self.chaser_name}] SetEntityState success: {fut.result().success}")
            else:
                self.get_logger().warn(f"[{self.chaser_name}] SetEntityState call failed")

        future.add_done_callback(callback)


def main(args=None):
    rclpy.init(args=args)
    node = ChaserHCWDynamics()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
