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

        # service client 
        self.client = self.create_client(SetEntityState, '/set_entity_state')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /set_entity_state...')
        self.get_logger().info('/set_entity_state READY')

        # Hill/Clohessy-Wiltshire initial parameters 
        self.declare_parameter('n', 0.2) # rad/s (same as target.orbital_rate)
        self.declare_parameter('dt', 0.02)
        self.declare_parameter('x0', 0.5) # m radial offset
        self.declare_parameter('y0', 0.0)
        self.declare_parameter('z0', 0.0)
        self.declare_parameter('xdot0', 0.0)
        self.declare_parameter('ydot0', 0.0)
        self.declare_parameter('zdot0', 0.0)

        self.n  = self.get_parameter('n').value
        self.dt = self.get_parameter('dt').value

        self.x0  = self.get_parameter('x0').value
        self.y0  = self.get_parameter('y0').value
        self.z0  = self.get_parameter('z0').value
        self.xdot0 = self.get_parameter('xdot0').value
        self.ydot0 = self.get_parameter('ydot0').value
        self.zdot0 = self.get_parameter('zdot0').value

        self.target_name = 'target'
        self.chaser_name = 'chaser'
        self.target_pose  = None
        self.target_twist = None
        self.t = 0.0                     # HCW time (starts when target is seen)
        self.first_update = True

        # create state subscription
        self.model_sub = self.create_subscription(
            ModelStates, '/model_states', self.model_cb, 10)

        # timer will be created inside model_cb the first time we see the target

    # /model_states callback 
    def model_cb(self, msg: ModelStates):
        try:
            idx = msg.name.index(self.target_name)
            pose  = msg.pose[idx]
            twist = msg.twist[idx]
        except ValueError:
            # target not yet in the list
            return

        # initialize timer the first time we see the target 
        if self.target_pose is None:
            self.get_logger().info(f"Target '{self.target_name}' detected → starting HCW timer")
            self.target_pose  = pose
            self.target_twist = twist

            # send initial state so Gazebo knows the chaser exists
            self.send_initial_state()

            # now create the periodic timer
            self.timer = self.create_timer(self.dt, self.step)
            return

        # after first message only keep the last target state 
        self.target_pose  = pose
        self.target_twist = twist

    # send intiial chaser state to Gazebo 
    def send_initial_state(self):
        target_pos = np.array([self.target_pose.position.x,
                               self.target_pose.position.y,
                               self.target_pose.position.z])
        target_vel = np.array([self.target_twist.linear.x,
                               self.target_twist.linear.y,
                               self.target_twist.linear.z])

        # LVLH basis for when target appears (local vertical local horizontal) 
        r_vec = target_pos - np.array([0.0, 0.0, target_pos[2]]) 
        R = np.linalg.norm(r_vec)
        unit_r = r_vec / R if R > 1e-6 else np.array([1.0, 0.0, 0.0])

        v_norm = np.linalg.norm(target_vel)
        unit_theta = target_vel / v_norm if v_norm > 1e-6 else np.array([0.0, 1.0, 0.0])

        unit_h = np.cross(unit_r, unit_theta)
        h_norm = np.linalg.norm(unit_h)
        unit_h = unit_h / h_norm if h_norm > 1e-6 else np.array([0.0, 0.0, 1.0])

        # desired relative offset in lvlh 
        rel_pos_lvlh = np.array([self.x0, self.y0, self.z0])
        offset_pos   = (rel_pos_lvlh[0] * unit_r +
                        rel_pos_lvlh[1] * unit_theta +
                        rel_pos_lvlh[2] * unit_h)

        rel_vel_lvlh = np.array([self.xdot0, self.ydot0, self.zdot0])
        offset_vel   = (rel_vel_lvlh[0] * unit_r +
                        rel_vel_lvlh[1] * unit_theta +
                        rel_vel_lvlh[2] * unit_h)

        chaser_pos = target_pos + offset_pos
        chaser_vel = target_vel + offset_vel

        # build entityState to send 
        pose_msg = Pose()
        pose_msg.position = Point(x=chaser_pos[0], y=chaser_pos[1], z=chaser_pos[2])
        pose_msg.orientation = Quaternion(w=1.0)

        twist_msg = Twist()
        twist_msg.linear = Vector3(x=chaser_vel[0], y=chaser_vel[1], z=chaser_vel[2])

        state = EntityState()
        state.name = self.chaser_name
        state.pose = pose_msg
        state.twist = twist_msg
        state.reference_frame = 'world'

        req = SetEntityState.Request()
        req.state = state
        self.client.call_async(req)
        self.get_logger().info("Initial chaser state sent (relative offset applied)")       

    # HCW closed-form + LVLH -> world transformation
    def step(self):
        if self.target_pose is None or self.target_twist is None:
            return

        # HCW soln in rel coords 
        c = np.cos(self.n * self.t)
        s = np.sin(self.n * self.t)

        x = (4 - 3*c)*self.x0 + (s/self.n)*self.xdot0 + (2/self.n)*(1-c)*self.ydot0
        y = 6*(s - self.n*self.t)*self.x0 + self.y0 \
            - (2/self.n)*(1-c)*self.xdot0 \
            + (1/self.n)*(4*s - 3*self.n*self.t)*self.ydot0
        z = self.z0*c + (self.zdot0/self.n)*s

        # relative velocities (derivatives)
        xd = 3*self.n*s*self.x0 + c*self.xdot0 + 2*s*self.ydot0
        yd = 6*self.n*(c-1)*self.x0 - 2*s*self.xdot0 + (4*c - 3)*self.ydot0
        zd = -self.z0*self.n*s + self.zdot0*c

        # current target state
        tp = np.array([self.target_pose.position.x,
                       self.target_pose.position.y,
                       self.target_pose.position.z])
        tv = np.array([self.target_twist.linear.x,
                       self.target_twist.linear.y,
                       self.target_twist.linear.z])

        # lvlh basis at this instant 
        r_vec = tp - np.array([0.0, 0.0, tp[2]])
        R = np.linalg.norm(r_vec)
        unit_r = r_vec / R if R > 1e-6 else np.array([1.0, 0.0, 0.0])

        v_norm = np.linalg.norm(tv)
        unit_theta = tv / v_norm if v_norm > 1e-6 else np.array([0.0, 1.0, 0.0])

        unit_h = np.cross(unit_r, unit_theta)
        h_norm = np.linalg.norm(unit_h)
        unit_h = unit_h / h_norm if h_norm > 1e-6 else np.array([0.0, 0.0, 1.0])

        # tansform relative to world 
        rel_pos = np.array([x, y, z])
        rel_vel = np.array([xd, yd, zd])

        world_pos_offset = (rel_pos[0] * unit_r +
                            rel_pos[1] * unit_theta +
                            rel_pos[2] * unit_h)
        world_vel_offset = (rel_vel[0] * unit_r +
                            rel_vel[1] * unit_theta +
                            rel_vel[2] * unit_h)

        chaser_pos = tp + world_pos_offset
        chaser_vel = tv + world_vel_offset

        # send new state to Gazebo for simulation
        pose_msg = Pose()
        pose_msg.position = Point(x=chaser_pos[0], y=chaser_pos[1], z=chaser_pos[2])
        pose_msg.orientation = Quaternion(w=1.0)

        twist_msg = Twist()
        twist_msg.linear = Vector3(x=chaser_vel[0], y=chaser_vel[1], z=chaser_vel[2])

        state = EntityState()
        state.name = self.chaser_name
        state.pose = pose_msg
        state.twist = twist_msg
        state.reference_frame = 'world'

        req = SetEntityState.Request()
        req.state = state
        self.client.call_async(req)

        # print chaser absolute position
        self.get_logger().info(
            f"[chaser] t={self.t:6.2f}s  abs=({chaser_pos[0]:7.3f}, {chaser_pos[1]:7.3f}, {chaser_pos[2]:7.3f})"
        )

        # time update 
        self.t += self.dt
        if self.first_update:
            self.first_update = False

def main(args=None):
    rclpy.init(args=args)
    node = ChaserHCWDynamics()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()