#!/usr/bin/env python3


from ament_index_python.packages import get_package_share_directory
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import json
import math
import os
import threading
import time
import rclpy

# Configuration
ARM_L_JOINTS = ['shoulder_l_joint', 'elbow_l_joint', 'wrist_l_joint']
ARM_R_JOINTS = ['shoulder_r_joint', 'elbow_r_joint', 'wrist_r_joint']
ARM_LIMIT = 1.5707
ARM_SETTLE = 1.0

# Navigation - robust waypoint reaching
POS_TOLERANCE = 0.12     # metres 
YAW_TOLERANCE = 0.20     # radians 
NAV_SPEED = 0.5         # m/s 
NAV_ANGULAR_SPEED = 1.0 # rad/s 
NAV_TIMEOUT = 120.0      # seconds per waypoint


def wrap_angle(angle):
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle


class NavNode(Node):
    def __init__(self, traj_path):
        super().__init__('navigation_node')
        # Publishers
        self.cmd_pub = self.create_publisher(Twist, '/cmd', 10)
        self.arm_l_pub = self.create_publisher(JointTrajectory, '/arm_left_controller/joint_trajectory', 10)
        self.arm_r_pub = self.create_publisher(JointTrajectory, '/arm_right_controller/joint_trajectory', 10)
        self.grip_l_pub = self.create_publisher(Float64MultiArray, '/gripper_left_controller/commands', 10)
        self.grip_r_pub = self.create_publisher(Float64MultiArray, '/gripper_right_controller/commands', 10)
        # Subscribers
        self.create_subscription(Odometry, '/odom', self._odom_cb, 10)
        # State
        self._pose_lock = threading.Lock()
        self._x = 0.0
        self._y = 0.0
        self._yaw = 0.0
        self._odom_ready = False
        self.is_running = True
        self.loc = {'x': 0.0, 'y': 0.0, 'yaw': 0.0}
        # Load trajectory
        with open(traj_path) as f:
            self.traj = json.load(f)
        n = len(self.traj.get('waypoints', []))
        self.get_logger().info(f'Loaded trajectory: {traj_path} ({n} waypoints)')
    
    def _odom_cb(self, msg):
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        #self.get_logger().info(f'Odom: x={msg.pose.pose.position.x:.2f}, y={msg.pose.pose.position.y:.2f}, yaw={math.degrees(yaw):.1f}°')
        self.loc = {'x': msg.pose.pose.position.x, 'y': msg.pose.pose.position.y, 'yaw': yaw}
        with self._pose_lock:
            self._x = msg.pose.pose.position.x
            self._y = msg.pose.pose.position.y
            self._yaw = yaw
            self._odom_ready = True
    
    def _pose(self):
        with self._pose_lock:
            return self._x, self._y, self._yaw
    
    def _stop(self):
        self.cmd_pub.publish(Twist())
    
    def _send_twist(self, linear_x, angular_z):
        cmd = Twist()
        cmd.linear.x = linear_x
        cmd.angular.z = angular_z
        self.cmd_pub.publish(cmd)
    
    def _send_arm(self, side, positions):
        msg = JointTrajectory()
        msg.joint_names = ARM_L_JOINTS if side == 'l' else ARM_R_JOINTS
        pt = JointTrajectoryPoint()
        pt.positions = [max(-ARM_LIMIT, min(ARM_LIMIT, float(p))) for p in positions]
        pt.velocities = [0.0] * 3
        pt.time_from_start = Duration(sec=1)
        msg.points = [pt]
        pub = self.arm_l_pub if side == 'l' else self.arm_r_pub
        pub.publish(msg)
    
    def _send_grip(self, side, positions):
        
        msg = Float64MultiArray()
        msg.data = [float(p) for p in positions]
        pub = self.grip_l_pub if side == 'l' else self.grip_r_pub
        pub.publish(msg)
    
    def _navigate_to(self, tx, ty, tyaw):
        t0 = time.monotonic()
        
        while self.is_running and time.monotonic() - t0 < NAV_TIMEOUT:
            if self.loc.get('x') > 2:
                self.get_logger().info('In Room 2 (Living Room)')
            else:
                self.get_logger().info('In Room 1 (Office)')
            x, y, yaw = self._pose()
            
            dx = tx - x
            dy = ty - y
            dist = math.hypot(dx, dy)
            
            # Check arrival
            if dist < POS_TOLERANCE and abs(wrap_angle(tyaw - yaw)) < YAW_TOLERANCE:
                self._stop()
                return True
            
            # Drive toward waypoint
            if dist >= POS_TOLERANCE:
                target_hdg = math.atan2(dy, dx)
                hdg_err = wrap_angle(target_hdg - yaw)
                
                if abs(hdg_err) < 0.05:
                    hdg_err = 0.0
                if abs(hdg_err) > 0.40:
                    forward_speed = NAV_SPEED * 0.15
                    angular_cmd = math.copysign(NAV_ANGULAR_SPEED * 0.5, hdg_err)
                else:
                    forward_speed = NAV_SPEED * min(1.0, dist / 0.6)
                    angular_cmd = max(-NAV_ANGULAR_SPEED * 0.4, min(NAV_ANGULAR_SPEED * 0.4, hdg_err * 0.25))
                self._send_twist(forward_speed, angular_cmd)
            else:
                # Fine-tune yaw at the waypoint (very gentle)
                yaw_err = wrap_angle(tyaw - yaw)
                if abs(yaw_err) > YAW_TOLERANCE:
                    self._send_twist(0.0, math.copysign(NAV_ANGULAR_SPEED * 0.25, yaw_err))
                else:
                    self._stop()
            time.sleep(0.05)
        self._stop()
        return False
    
    def play(self):

        # Wait for odometry
        self.get_logger().info('Waiting for odometry...')
        while self.is_running:
            with self._pose_lock:
                if self._odom_ready:
                    break
            time.sleep(0.1)
        x, y, _ = self._pose()
        self.get_logger().info(f'Start: ({x:.2f}, {y:.2f})\n')
        waypoints = self.traj.get('waypoints', [])
        total = len(waypoints)
        self.get_logger().info(f'Navigation: {total} waypoints\n')
        for idx, wp in enumerate(waypoints):
            if not self.is_running:
                break
            wtype = wp.get('type', 'pose')
            x, y, _ = self._pose()
            if wtype == 'pose':
                tx, ty, tyaw = wp['x'], wp['y'], wp['yaw']
                self.get_logger().info(f'[{idx + 1}/{total}] Going to ({tx:.2f}, {ty:.2f})...')
                ok = self._navigate_to(tx, ty, tyaw)
                if ok:
                    self.get_logger().info(f'  Reached ({x:.2f}, {y:.2f})\n')
                else:
                    x, y, _ = self._pose()
                    self.get_logger().warn(f'  Timeout at ({x:.2f}, {y:.2f})\n')
                if 'arm_l' in wp:
                    self._send_arm('l', wp['arm_l'])
                    self.get_logger().info(f'  Arm L: {[round(v, 2) for v in wp["arm_l"]]}')
                if 'arm_r' in wp:
                    self._send_arm('r', wp['arm_r'])
                    self.get_logger().info(f'  Arm R: {[round(v, 2) for v in wp["arm_r"]]}')
                if 'arm_l' in wp or 'arm_r' in wp:
                    time.sleep(ARM_SETTLE)
                if 'grip_l' in wp:
                    self._send_grip('l', wp['grip_l'])
                    self.get_logger().info(f'  Gripper L')
                if 'grip_r' in wp:
                    self._send_grip('r', wp['grip_r'])
                    self.get_logger().info(f'  Gripper R')
                
                if any(k in wp for k in ['arm_l', 'arm_r', 'grip_l', 'grip_r']):
                    self.get_logger().info('')
        self._stop()
        x, y, _ = self._pose()
        self.get_logger().info(f'Done. Final: ({x:.2f}, {y:.2f})')

def main(args=None):
    rclpy.init(args=args)
    traj_dir = os.path.join(
        get_package_share_directory('robot_description'), 'trajectories')
    traj_path = os.path.join(traj_dir, 'traj.json')
    if not os.path.exists(traj_path):
        print(f'ERROR: {traj_path} not found!')
        return
    node = NavNode(traj_path)
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
    try:
        node.play()
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted.')
    finally:
        node.is_running = False
        node._stop()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()