#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SetEntityState
from gazebo_msgs.msg import EntityState
from geometry_msgs.msg import Pose, Twist
import numpy as np

class TargetDynamics(Node):
    def __init__(self):
        super().__init__('target_dynamics')

        # Connect to Gazebo service
        self.client = self.create_client(SetEntityState, '/gazebo/set_entity_state')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /gazebo/set_entity_state...')

        # Simulation parameters
        self.declare_parameter('name', 'target')
        self.declare_parameter('omega', [0.0, 0.0, 0.05])  # rad/s
        self.declare_parameter('radius', 2.0)              # meters
        self.declare_parameter('orbital_rate', 0.1)        # rad/s
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

        # Simple orbital motion (circular)
        x = self.radius * np.cos(self.orbital_rate * self.t)
        y = self.radius * np.sin(self.orbital_rate * self.t)
        z = 2.0

        # Simple rotation around z-axis
        yaw = self.omega[2] * self.t
        q = self.euler_to_quat(0.0, 0.0, yaw)

        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z
        pose.orientation.x = q[0]
        pose.orientation.y = q[1]
        pose.orientation.z = q[2]
        pose.orientation.w = q[3]

        twist = Twist()
        twist.linear.x = -self.radius * self.orbital_rate * np.sin(self.orbital_rate * self.t)
        twist.linear.y =  self.radius * self.orbital_rate * np.cos(self.orbital_rate * self.t)

        state = EntityState()
        state.name = self.entity_name
        state.pose = pose
        state.twist = twist
        state.reference_frame = 'world'

        req = SetEntityState.Request()
        req.state = state
        self.client.call_async(req)

    def euler_to_quat(self, roll, pitch, yaw):
        cy = np.cos(yaw * 0.5)
        sy = np.sin(yaw * 0.5)
        cp = np.cos(pitch * 0.5)
        sp = np.sin(pitch * 0.5)
        cr = np.cos(roll * 0.5)
        sr = np.sin(roll * 0.5)
        q = np.array([
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy,
        ])
        return q

def main(args=None):
    rclpy.init(args=args)
    node = TargetDynamics()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
