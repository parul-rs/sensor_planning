#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SetEntityState
from gazebo_msgs.msg import EntityState
from geometry_msgs.msg import Pose, Twist, Vector3, Quaternion, Point
import numpy as np

class TargetDynamics(Node):
    def __init__(self):
        super().__init__('target_dynamics')

        # CORRECT SERVICE NAME
        self.client = self.create_client(SetEntityState, '/set_entity_state')

        # Wait for service
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /set_entity_state...')

        self.get_logger().info('/set_entity_state is READY')

        # Parameters
        self.declare_parameter('name', 'target')
        self.declare_parameter('omega', [0.0, 0.0, 0.1])
        self.declare_parameter('radius', 2.0)
        self.declare_parameter('orbital_rate', 0.2)
        self.declare_parameter('dt', 0.05)

        self.entity_name = self.get_parameter('name').value
        self.omega = np.array(self.get_parameter('omega').value)
        self.radius = self.get_parameter('radius').value
        self.orbital_rate = self.get_parameter('orbital_rate').value
        self.dt = self.get_parameter('dt').value

        self.t = 0.0
        self.timer = self.create_timer(self.dt, self.step)

    def step(self):
        self.t += self.dt

        x = self.radius * np.cos(self.orbital_rate * self.t)
        y = self.radius * np.sin(self.orbital_rate * self.t)
        z = 4.0
        yaw = self.omega[2] * self.t
        q = self.euler_to_quat(0.0, 0.0, yaw)

        pose = Pose()
        pose.position = Point(x=x, y=y, z=z)
        pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

        twist = Twist()
        twist.linear = Vector3(
            x=-self.radius * self.orbital_rate * np.sin(self.orbital_rate * self.t),
            y=self.radius * self.orbital_rate * np.cos(self.orbital_rate * self.t),
            z=0.0
        )

        state = EntityState()
        state.name = self.entity_name
        state.pose = pose
        state.twist = twist
        state.reference_frame = 'world'

        req = SetEntityState.Request()
        req.state = state
        self.client.call_async(req)

    def euler_to_quat(self, roll, pitch, yaw):
        cy, sy = np.cos(yaw * 0.5), np.sin(yaw * 0.5)
        cp, sp = np.cos(pitch * 0.5), np.sin(pitch * 0.5)
        cr, sr = np.cos(roll * 0.5), np.sin(roll * 0.5)
        return np.array([
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy,
        ])

def main():
    rclpy.init()
    node = TargetDynamics()
    rclpy.spin(node)

if __name__ == '__main__':
    main()