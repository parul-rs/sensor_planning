#!/usr/bin/env python3
"""
Chaser HCW (Hill Clohessy-Wiltshire) dynamics: Gz Harmonic version.
 
Changes from Classic version:
Subscribes to /model/target/pose (geometry_msgs/Pose, bridged by ros_gz_bridge)
    instead of /model_states (which does not exist in Gz Harmonic).
Target velocity is estimated via finite differences on the bridged pose.
Poses are NOT pushed via gz-transport /world/<world>/set_pose instead of
    the gazebo_msgs SetEntityState service- this didn't work now we are doing: 
    Poses are pushed by the gz_service CLI subprocess calling /world/world/set_pose
    because the gz-transport python bindings aren't available via standard 
    apt for jazzy/noble
"""
import rclpy
from rclpy.node import Node
import numpy as np
from geometry_msgs.msg import Pose, Twist, Vector3, Quaternion, Point

class ChaserHCWDynamics(Node):
    def __init__(self):
        super().__init__('chaser_hcw_dynamics')

        # Hill/Clohessy-Wiltshire initial parameters 
        self.declare_parameter('n', 0.2) # rad/s (same as target.orbital_rate)
        self.declare_parameter('dt', 0.08)
        self.declare_parameter('x0', 0.5) # m radial offset
        self.declare_parameter('y0', 0.0)
        self.declare_parameter('z0', 0.0)
        self.declare_parameter('xdot0', 0.0)
        self.declare_parameter('ydot0', 0.0)
        self.declare_parameter('zdot0', 0.0)
        self.declare_parameter('world_name', 'empty_no_floor')
        self.declare_parameter('safety_radius', 0.1) # meters

        self.n  = self.get_parameter('n').value
        self.dt = self.get_parameter('dt').value

        self.x0  = self.get_parameter('x0').value
        self.y0  = self.get_parameter('y0').value
        self.z0  = self.get_parameter('z0').value
        self.xdot0 = self.get_parameter('xdot0').value
        self.ydot0 = self.get_parameter('ydot0').value
        self.zdot0 = self.get_parameter('zdot0').value
        self.world  = self.get_parameter('world_name').value
        self.safety = self.get_parameter('safety_radius').value
        self.target_name = 'target'
        self.chaser_name = 'chaser'
        # self.target_pose  = None
        # self.target_twist = None
        self.t = 0.0    # HCW time (starts when target is seen)
        self.first_update = True

        # self.model_sub = self.create_subscription(
        #     ModelStates, '/model_states', self.model_cb, 10)

        self.target_pose_msg = None # last geometry_msgs/Pose from bridge
        self.target_pos      = None # np.array
        self.target_vel      = np.zeros(3) # estimated via finite diff
        self._prev_pos       = None
        self._prev_t         = None
        self.init_rel_pos    = None
        self.init_rel_vel    = None

        self.service_name = f'/world/{self.world}/set_pose'
 
        # ROS2 subscriptions
        # /model/target/pose is bridged from gz by ros_gz_bridge
        self.create_subscription(
            Pose,
            '/model/target/pose',
            self._target_pose_cb,
            10
        )

        self.pose_pub = self.create_publisher(Pose, '/chaser_pose_cmd', 1)
 
        self.get_logger().info(
            'ChaserHCWDynamics waiting for set_pose ...'
        )

    def _target_pose_cb(self, msg: Pose):
        now = self.get_clock().now().nanoseconds * 1e-9
        pos = np.array([msg.position.x, msg.position.y, msg.position.z])

        #finite diff velocity estimate 
        if self._prev_pos is not None and self._prev_t is not None:
            dt_obs = now - self._prev_t
            if dt_obs > 1e-3:
                self.target_vel = (pos - self._prev_pos) / dt_obs
        self._prev_pos = pos.copy()
        self._prev_t = now 
        self.target_pos = pos
        self.target_pose_msg = msg 
        if self.first_update:
            self.get_logger().info("Target pose received, initializing chaser.")
            self._initialize_chaser()
            self.create_timer(self.dt, self._step)
            self.first_update = False 
    
    def _initialize_chaser(self):
        tp = self.target_pos
        tv = self.target_vel

        unit_r, unit_theta, unit_h, omega_vec = self._lvlh_basis(tp,tv)

        rel_pos_lvlh = np.array([self.x0, self.y0, self.z0])
        rel_vel_lvlh = np.array([self.xdot0, self.ydot0, self.zdot0])

        offset_pos = self._lvlh_to_world(rel_pos_lvlh, unit_r, unit_theta, unit_h)
        offset_vel = self._lvlh_to_world(rel_vel_lvlh, unit_r, unit_theta, unit_h)
        offset_vel += np.cross(omega_vec, offset_pos) # rotating-frame term
 
        self.init_rel_pos = rel_pos_lvlh.copy()
        self.init_rel_vel = rel_vel_lvlh.copy()
        self.t = 0.0
 
        chaser_pos = tp + offset_pos
        # self._set_gz_pose(self.chaser_name, chaser_pos, [0.0, 0.0, 0.0, 1.0])
        msg = Pose()
        msg.position.x = float(chaser_pos[0])
        msg.position.y = float(chaser_pos[1])
        msg.position.z = float(chaser_pos[2])
        msg.orientation.x = 0.0
        msg.orientation.y = 0.0
        msg.orientation.z = 0.0
        msg.orientation.w = 1.0
        self.pose_pub.publish(msg)
        self.get_logger().info(f'Chaser initial position: {chaser_pos.round(3)}')   

        self.get_logger().info(
            f'Chaser initial position: {chaser_pos.round(3)}'
        )
 
    # /model_states callback 
    # def model_cb(self, msg: ModelStates):
    #     try:
    #         idx = msg.name.index(self.target_name)
    #         pose  = msg.pose[idx]
    #         twist = msg.twist[idx]
    #     except ValueError:
    #         return

    #     # initialize timer the first time we see the target 
    #     if self.target_pose is None:
    #         self.get_logger().info(f"Target '{self.target_name}' detected → starting HCW timer")
    #         self.target_pose  = pose
    #         self.target_twist = twist

    #         self.send_initial_state()
    #         self.timer = self.create_timer(self.dt, self.step)
    #         return

    #     # after first message only keep the last target state 
    #     self.target_pose  = pose
    #     self.target_twist = twist   

    # HCW closed-form + LVLH -> world transformation
    def _step(self):
        if self.target_pos is None:
            return
        
        x0, y0, z0 = self.init_rel_pos
        xdot0, ydot0, zdot0 = self.init_rel_vel

        c = np.cos(self.n * self.t)
        s = np.sin(self.n * self.t)

        # HCW relative position/velocity in LVLH
        x = (4 - 3*c)*x0 + (s/self.n)*xdot0 + (2/self.n)*(1-c)*ydot0
        y = 6*(s - self.n*self.t)*x0 + y0 \
            - (2/self.n)*(1-c)*xdot0 \
            + (1/self.n)*(4*s - 3*self.n*self.t)*ydot0
        z = z0*c + (zdot0/self.n)*s

        xd = 3*self.n*s*x0 + c*xdot0 + 2*s*ydot0
        yd = 6*self.n*(c-1)*x0 - 2*s*xdot0 + (4*c - 3)*ydot0
        zd = -z0*self.n*s + zdot0*c

        tp = self.target_pos
        tv = self.target_vel

        unit_r, unit_theta, unit_h, omega_vec = self._lvlh_basis(tp, tv)
        if unit_r is None: 
            return 

        # LVLH basis at current target pose -- moved into helper function
        # r_vec = tp.copy()
        # R = np.linalg.norm(r_vec)
        # if R < 1e-9:
        #     self.get_logger().error("Target at origin — skipping update")
        #     return
        # unit_r = r_vec / R

        # h_vec = np.cross(r_vec, tv)
        # h_norm = np.linalg.norm(h_vec)
        # if h_norm < 1e-6:
        #     unit_h = np.array([0.0, 0.0, 1.0])
        #     omega_vec = self.n * unit_h
        # else:
        #     unit_h = h_vec / h_norm
        #     omega_vec = h_vec / (R**2)

        # unit_theta = np.cross(unit_h, unit_r)

        # transform relative to world
        rel_pos = np.array([x, y, z])
        rel_vel = np.array([xd, yd, zd])

        world_pos_offset = (rel_pos[0] * unit_r +
                            rel_pos[1] * unit_theta +
                            rel_pos[2] * unit_h)
        world_vel_offset = (rel_vel[0] * unit_r +
                            rel_vel[1] * unit_theta +
                            rel_vel[2] * unit_h)
        
        # include rotation of LVLH frame
        world_vel_offset += np.cross(omega_vec, world_pos_offset)

        # safety clamp so chaser doesn't collide with target 
        sep = np.linalg.norm(world_pos_offset)
        if sep < self.safety:
            self.get_logger().warn(
                f'Safety clamp: separation {sep:.3f} m < {self.safety:.3f} m'
            )
            world_pos_offset = (world_pos_offset / (sep + 1e-9)) * self.safety

        chaser_pos = tp + world_pos_offset
        chaser_vel = tv + world_vel_offset

        #self._set_gz_pose(self.chaser_name, chaser_pos, [0.0, 0.0, 0.0, 1.0])
        msg = Pose()
        msg.position.x = float(chaser_pos[0])
        msg.position.y = float(chaser_pos[1])
        msg.position.z = float(chaser_pos[2])
        msg.orientation.x = 0.0
        msg.orientation.y = 0.0
        msg.orientation.z = 0.0
        msg.orientation.w = 1.0
        self.pose_pub.publish(msg)
 
        self.get_logger().info(
            f'[chaser] t={self.t:6.2f}s  '
            f'pos=({chaser_pos[0]:7.3f}, {chaser_pos[1]:7.3f}, {chaser_pos[2]:7.3f})  '
            f'sep={sep:.3f} m'
        )

        self.t += self.dt
        if self.first_update:
            self.first_update = False

    # LVLH helpers 
    def _lvlh_basis(self, tp, tv):
        """Return (unit_r, unit_theta, unit_h, omega_vec) or None tuple on failure."""
        R = np.linalg.norm(tp)
        if R < 1e-9:
            self.get_logger().error('Target at origin, skipping update.')
            return None, None, None, None
 
        unit_r = tp / R
        h_vec  = np.cross(tp, tv)
        h_norm = np.linalg.norm(h_vec)
 
        if h_norm < 1e-6:
            unit_h    = np.array([0.0, 0.0, 1.0])
            omega_vec = self.n * unit_h
        else:
            unit_h    = h_vec / h_norm
            omega_vec = h_vec / (R**2)
 
        unit_theta = np.cross(unit_h, unit_r)
        return unit_r, unit_theta, unit_h, omega_vec

    def _lvlh_to_world(self, vec_lvlh, unit_r, unit_theta, unit_h):
        return (vec_lvlh[0] * unit_r +
                vec_lvlh[1] * unit_theta +
                vec_lvlh[2] * unit_h)
    
    # def _set_gz_pose(self, name: str, pos, quat) -> bool:
    #         req = (
    #             f'name: "{name}", '
    #             f'position: {{x: {pos[0]}, y: {pos[1]}, z: {pos[2]}}}, '
    #             f'orientation: {{x: {quat[0]}, y: {quat[1]}, z: {quat[2]}, w: {quat[3]}}}'
    #         )
    #         cmd = [
    #             'gz', 'service', '-s', self.service_name,
    #             '--reqtype', 'gz.msgs.Pose',
    #             '--reptype', 'gz.msgs.Boolean',
    #             '--timeout', str(self.gz_timeout_ms),
    #             '--req', req,
    #         ]
    #         try:
    #             result = subprocess.run(
    #                 cmd, capture_output=True, text=True,
    #                 timeout=(self.gz_timeout_ms / 1000.0) + 0.1
    #             )
    #             if result.returncode != 0:
    #                 self.get_logger().warn(f'gz service call failed: {result.stderr.strip()}')
    #                 return False
    #             return True
    #         except subprocess.TimeoutExpired:
    #             self.get_logger().warn('gz service call timed out')
    #             return False 

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