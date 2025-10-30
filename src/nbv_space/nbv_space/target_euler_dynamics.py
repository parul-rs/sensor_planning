#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
from gazebo_msgs.srv import SetEntityState
from gazebo_msgs.msg import EntityState
from geometry_msgs.msg import Pose, Twist, Vector3, Quaternion, Point

# notes from pete 
# lower earth orbit - 800 km 
# hcw - good for a 15 minute - for longer, we woudl propagate the full two body problem , 
# dont need to do the three body problem unless we'll be in a cislunar region 

def quat_mul(q1, q2):
    x1,y1,z1,w1 = q1
    x2,y2,z2,w2 = q2
    return np.array([
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
        w1*w2 - x1*x2 - y1*y2 - z1*z2
    ])

def quat_normalize(q):
    return q / np.linalg.norm(q)

class TargetEulerDynamics(Node):
    def __init__(self):
        super().__init__('target_euler_dynamics')

        self.client = self.create_client(SetEntityState, '/set_entity_state')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /set_entity_state...')
        self.get_logger().info('/set_entity_state READY')

        self.declare_parameter('name', 'target')
        self.declare_parameter('I', [1.0, 1.0, 1.0])
        self.declare_parameter('omega0', [0.0, 0.0, 0.1])
        self.declare_parameter('q0', [0.0, 0.0, 0.0, 1.0])
        self.declare_parameter('dt', 0.02)
        self.declare_parameter('orbital_radius', 2.0)
        self.declare_parameter('orbital_rate', 0.2)
        self.declare_parameter('z', 4.0)

        self.entity_name = self.get_parameter('name').value
        Idiag = np.array(self.get_parameter('I').value, dtype=float)
        self.I = np.diag(Idiag)
        self.Iinv = np.linalg.inv(self.I)
        self.omega = np.array(self.get_parameter('omega0').value, dtype=float)
        self.q = quat_normalize(np.array(self.get_parameter('q0').value, dtype=float))
        self.dt = float(self.get_parameter('dt').value)

        self.R = float(self.get_parameter('orbital_radius').value)
        self.n = float(self.get_parameter('orbital_rate').value)
        self.z = float(self.get_parameter('z').value)
        self.t = 0.0

        self.timer = self.create_timer(self.dt, self.step)

    def step(self):
        tau = np.zeros(3)
        omega_cross = np.cross(self.omega, self.I @ self.omega)
        omega_dot = self.Iinv @ (-omega_cross + tau)
        omega_mid = self.omega + 0.5 * self.dt * omega_dot
        omega_dot_mid = self.Iinv @ (-np.cross(omega_mid, self.I @ omega_mid) + tau)
        self.omega += self.dt * omega_dot_mid

        omega_quat = np.array([self.omega[0], self.omega[1], self.omega[2], 0.0])
        q_dot = 0.5 * quat_mul(self.q, omega_quat)
        self.q = quat_normalize(self.q + self.dt * q_dot)

        self.t += self.dt
        x = self.R * np.cos(self.n * self.t)
        y = self.R * np.sin(self.n * self.t)
        z = self.z
        vx = -self.R * self.n * np.sin(self.n * self.t)
        vy =  self.R * self.n * np.cos(self.n * self.t)

        pose = Pose()
        pose.position = Point(x=x, y=y, z=z)
        pose.orientation = Quaternion(x=self.q[0], y=self.q[1], z=self.q[2], w=self.q[3])

        twist = Twist()
        twist.linear = Vector3(x=vx, y=vy, z=0.0)
        twist.angular = Vector3(x=self.omega[0], y=self.omega[1], z=self.omega[2])

        state = EntityState()
        state.name = self.entity_name
        state.pose = pose
        state.twist = twist
        state.reference_frame = 'world'

        req = SetEntityState.Request()
        req.state = state
        self.client.call_async(req)

def main(args=None):
    rclpy.init(args=args)
    node = TargetEulerDynamics()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
