#!/usr/bin/env python3
"""
drift_asg Teleop + Trajectory Recorder
=======================================
DRIVE  : wasd = fwd/back/turn   space = stop
ARM-L  : rf=shoulder  tg=elbow  yh=wrist
ARM-R  : uj=shoulder  ik=elbow  ol=wrist
GRIPS  : zx = L open/close      cv = R open/close
PRESETS: 1=home  2=reach  3=carry  4=drop

RECORD : [  = start recording
           ]  = stop & save  ->  ~/drift_trajectories/traj_YYYYMMDD_HHMMSS.json
           m  = manual snapshot (force-save pose + arm state right now)

Waypoints are saved automatically every 10 stops so you never lose a session.
"""

import sys, tty, termios, threading, json, os, time, math
from datetime import datetime

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

# ── Robot constants ──────────────────────────────────────────────────────────
ARM_L_JOINTS = ['shoulder_l_joint', 'elbow_l_joint', 'wrist_l_joint']
ARM_R_JOINTS = ['shoulder_r_joint', 'elbow_r_joint', 'wrist_r_joint']
ARM_LIMIT    = 1.5707

# Gripper joint limits (from URDF, mimic removed — both fingers commanded directly)
#   ll_joint: lower=-0.03   upper=-0.0001   (moves negative to close)
#   lr_joint: lower=0.0001  upper=0.03      (moves positive to close)
#   rl_joint: lower=-0.03   upper=-0.0001   (moves negative to close)
#   rr_joint: lower=0.0001  upper=0.03      (moves positive to close)
GRIP_OPEN  = [ 0.0,     0.0]      # both fingers at open limit
GRIP_CLOSE = [-0.028,   0.028]    # both fingers near closed limit

PRESETS = {
    '1': ('HOME',  {'l': [0.0, 0.0,  0.0], 'r': [0.0, 0.0,  0.0]}),
    '2': ('REACH', {'l': [0.0, 1.2,  0.8], 'r': [0.0, 1.2,  0.8]}),
    '3': ('CARRY', {'l': [0.0, 0.5, -0.5], 'r': [0.0, 0.5, -0.5]}),
    '4': ('DROP',  {'l': [0.0, 1.4,  1.2], 'r': [0.0, 1.4,  1.2]}),
}

SAVE_DIR        = os.path.expanduser('~/drift_ws/src/robot_description/trajectories')
STOP_WAIT       = 0.35    # s  -- robot must be still this long before auto-snapshot
VEL_THRESHOLD   = 0.015   # m/s or rad/s -- "stopped" threshold
MIN_DIST        = 0.03    # m  -- minimum travel to record a new pose waypoint
MIN_YAW         = 0.05    # rad
AUTO_SAVE_EVERY = 10      # waypoints between incremental saves


class TeleopRecorder(Node):
    def __init__(self):
        super().__init__('teleop_recorder')

        # Publishers
        self.cmd_pub    = self.create_publisher(Twist,             '/cmd', 10)
        self.arm_l_pub  = self.create_publisher(JointTrajectory,   '/arm_left_controller/joint_trajectory',  10)
        self.arm_r_pub  = self.create_publisher(JointTrajectory,   '/arm_right_controller/joint_trajectory', 10)
        self.grip_l_pub = self.create_publisher(Float64MultiArray, '/gripper_left_controller/commands', 10)
        self.grip_r_pub = self.create_publisher(Float64MultiArray, '/gripper_right_controller/commands', 10)

        self.create_subscription(Odometry, '/odom', self._odom_cb, 10)

        self._lock  = threading.Lock()
        self._odom  = {'x': 0.0, 'y': 0.0, 'yaw': 0.0}
        self._vel   = {'linear': 0.0, 'angular': 0.0}
        self._arm_l = [0.0, 0.0, 0.0]
        self._arm_r = [0.0, 0.0, 0.0]

        # Teleop
        self.wheel_speed = 0.5
        self.arm_step    = 0.15

        # Recording
        self._recording  = False
        self._waypoints  = []
        self._last_snap  = None   # last recorded pose for dedup
        self._save_path  = None
        self._autosave_n = 0      # waypoint count at last autosave

        # Stop detection (updated inside _odom_cb with lock held)
        self._was_moving = False
        self._stop_ts    = None
        self._stop_done  = False  # waypoint already saved for this stop

    # ── Odometry callback ────────────────────────────────────────────────────
    def _odom_cb(self, msg):
        with self._lock:
            self._odom['x']   = msg.pose.pose.position.x
            self._odom['y']   = msg.pose.pose.position.y
            q = msg.pose.pose.orientation
            self._odom['yaw'] = math.atan2(
                2.0*(q.w*q.z + q.x*q.y), 1.0 - 2.0*(q.y*q.y + q.z*q.z))
            self._vel['linear']  = math.hypot(msg.twist.twist.linear.x,
                                               msg.twist.twist.linear.y)
            self._vel['angular'] = abs(msg.twist.twist.angular.z)
            if self._recording:
                self._detect_stop()

    def _detect_stop(self):
        """Called with lock held. Records a waypoint when robot stops after moving."""
        moving = (self._vel['linear']  > VEL_THRESHOLD or
                  self._vel['angular'] > VEL_THRESHOLD)
        if moving:
            self._was_moving = True
            self._stop_ts    = None
            self._stop_done  = False
            return

        if not self._was_moving:
            return  # sitting still since recording started — nothing to record

        if self._stop_ts is None:
            self._stop_ts = time.monotonic()
            return

        if not self._stop_done and time.monotonic() - self._stop_ts >= STOP_WAIT:
            self._append_pose()
            self._stop_done  = True
            self._was_moving = False  # reset so next move+stop triggers again

    # ── Snapshot helpers (always called with lock held) ───────────────────────
    def _snap(self):
        return {
            'x':     round(self._odom['x'],   4),
            'y':     round(self._odom['y'],   4),
            'yaw':   round(self._odom['yaw'], 4),
            'arm_l': [round(v, 4) for v in self._arm_l],
            'arm_r': [round(v, 4) for v in self._arm_r],
        }

    def _moved_enough(self, s):
        if self._last_snap is None:
            return True
        return (math.hypot(s['x'] - self._last_snap['x'],
                           s['y'] - self._last_snap['y']) >= MIN_DIST or
                abs(s['yaw'] - self._last_snap['yaw']) >= MIN_YAW)

    def _append_pose(self, force=False):
        s = self._snap()
        if not force and not self._moved_enough(s):
            return
        wp = {'type': 'pose', 'seq': len(self._waypoints) + 1, **s}
        self._waypoints.append(wp)
        self._last_snap = s
        print(f"  ({s['x']:.2f}, {s['y']:.2f})  yaw={math.degrees(s['yaw']):.0f} deg  seq={wp['seq']}")
        self._maybe_autosave()

    def _append_action(self, atype, data):
        if not self._recording:
            return
        s = self._snap()
        wp = {'type': atype, 'seq': len(self._waypoints) + 1,
              **s, 'data': [round(float(v), 4) for v in data]}
        self._waypoints.append(wp)
        self._last_snap = s
        print(f"  {atype}  {[round(v,2) for v in data]}  seq={wp['seq']}")
        self._maybe_autosave()

    def _maybe_autosave(self):
        if (self._save_path and
                len(self._waypoints) - self._autosave_n >= AUTO_SAVE_EVERY):
            self._write(self._save_path)
            self._autosave_n = len(self._waypoints)

    # ── Atomic file write ─────────────────────────────────────────────────────
    def _write(self, path):
        doc = {
            'metadata': {
                'created': datetime.now().isoformat(),
                'robot':   'drift_asg',
                'total':   len(self._waypoints),
            },
            'waypoints': self._waypoints,
        }
        tmp = path + '.tmp'
        with open(tmp, 'w') as f:
            json.dump(doc, f, indent=2)
        os.replace(tmp, path)   # atomic on POSIX — no corrupt files on crash

    # ── Recording controls ────────────────────────────────────────────────────
    def start_recording(self):
        with self._lock:
            if self._recording:
                print('  Already recording.')
                return
            self._waypoints  = []
            self._last_snap  = None
            self._was_moving = False
            self._stop_ts    = None
            self._stop_done  = False
            self._autosave_n = 0
            os.makedirs(SAVE_DIR, exist_ok=True)
            fname = f"traj_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json"
            self._save_path = os.path.join(SAVE_DIR, fname)
            self._recording  = True
        print('\033[91m REC  (stop robot = auto-snapshot,  m = force snapshot)\033[0m')

    def stop_recording(self):
        with self._lock:
            if not self._recording:
                return
            self._append_pose(force=True)   # capture final position
            self._recording = False
            path = self._save_path
            n    = len(self._waypoints)
        self._write(path)
        print(f'\033[92m SAVED  {path}  ({n} waypoints)\033[0m')

    def manual_snapshot(self):
        with self._lock:
            if not self._recording:
                print('  Not recording.')
                return
            self._append_pose(force=True)

    # ── Drive / arm / gripper ─────────────────────────────────────────────────
    def drive(self, lin, ang):
        t = Twist()
        t.linear.x  = float(lin * self.wheel_speed)
        t.angular.z = float(ang * self.wheel_speed)
        self.cmd_pub.publish(t)

    def stop_robot(self):
        self.cmd_pub.publish(Twist())
        print('  STOP')

    def move_arm(self, side, joint, delta):
        with self._lock:
            arm = list(self._arm_l if side == 'l' else self._arm_r)
        arm[joint] = max(-ARM_LIMIT, min(ARM_LIMIT, arm[joint] + delta))
        self._send_arm(side, arm)
        with self._lock:
            self._append_action(f'arm_{side}', arm)
        print(f'  ARM-{side.upper()}: {arm[0]:+.2f} {arm[1]:+.2f} {arm[2]:+.2f}')

    def set_preset(self, name, positions):
        self._send_arm('l', positions['l'])
        self._send_arm('r', positions['r'])
        with self._lock:
            if self._recording:
                s = self._snap()
                wp = {'type': 'preset', 'seq': len(self._waypoints) + 1,
                      **s, 'name': name,
                      'arm_l_cmd': positions['l'], 'arm_r_cmd': positions['r']}
                self._waypoints.append(wp)
                self._last_snap = s
                print(f"  preset:{name}  seq={wp['seq']}")
        print(f'  Preset: {name}')

    def open_grip(self, side):
        self._send_grip(side, GRIP_OPEN)
        with self._lock:
            self._append_action(f'grip_{side}', GRIP_OPEN)
        print(f'  Gripper {side.upper()} OPEN')

    def close_grip(self, side):
        self._send_grip(side, GRIP_CLOSE)
        with self._lock:
            self._append_action(f'grip_{side}', GRIP_CLOSE)
        print(f'  Gripper {side.upper()} CLOSED')

    def _send_arm(self, side, positions):
        positions = [max(-ARM_LIMIT, min(ARM_LIMIT, float(p))) for p in positions]
        msg = JointTrajectory()
        msg.joint_names = ARM_L_JOINTS if side == 'l' else ARM_R_JOINTS
        pt = JointTrajectoryPoint()
        pt.positions       = positions
        pt.velocities      = [0.0] * 3
        pt.time_from_start = Duration(sec=1)
        msg.points = [pt]
        (self.arm_l_pub if side == 'l' else self.arm_r_pub).publish(msg)
        with self._lock:
            if side == 'l': self._arm_l = positions
            else:           self._arm_r = positions

    def _send_grip(self, side, positions):
        # Both fingers are independent (mimic removed) — command both.
        msg = Float64MultiArray()
        msg.data = [float(p) for p in positions]
        (self.grip_l_pub if side == 'l' else self.grip_r_pub).publish(msg)


# ── Key handler ───────────────────────────────────────────────────────────────
def handle_key(key, node):
    if   key == 'w': node.drive( 1,  0)
    elif key == 's': node.drive(-1,  0)
    elif key == 'a': node.drive( 0,  1)
    elif key == 'd': node.drive( 0, -1)
    elif key == ' ': node.stop_robot()

    elif key == 'r': node.move_arm('l', 0,  node.arm_step)
    elif key == 'f': node.move_arm('l', 0, -node.arm_step)
    elif key == 't': node.move_arm('l', 1,  node.arm_step)
    elif key == 'g': node.move_arm('l', 1, -node.arm_step)
    elif key == 'y': node.move_arm('l', 2,  node.arm_step)
    elif key == 'h': node.move_arm('l', 2, -node.arm_step)

    elif key == 'u': node.move_arm('r', 0,  node.arm_step)
    elif key == 'j': node.move_arm('r', 0, -node.arm_step)
    elif key == 'i': node.move_arm('r', 1,  node.arm_step)
    elif key == 'k': node.move_arm('r', 1, -node.arm_step)
    elif key == 'o': node.move_arm('r', 2,  node.arm_step)
    elif key == 'l': node.move_arm('r', 2, -node.arm_step)

    elif key == 'z': node.open_grip('l')
    elif key == 'x': node.close_grip('l')
    elif key == 'c': node.open_grip('r')
    elif key == 'v': node.close_grip('r')
    elif key == 'b': node.open_grip('l');  node.open_grip('r')
    elif key == 'n': node.close_grip('l'); node.close_grip('r')

    elif key in PRESETS:
        name, pos = PRESETS[key]
        node.set_preset(name, pos)

    elif key == '[': node.start_recording()
    elif key == ']': node.stop_recording()
    elif key == 'm': node.manual_snapshot()

    elif key == '=':
        node.wheel_speed = min(10.0, node.wheel_speed + 0.5)
        print(f'  Speed: {node.wheel_speed:.1f}')
    elif key == '-':
        node.wheel_speed = max(0.5, node.wheel_speed - 0.5)
        print(f'  Speed: {node.wheel_speed:.1f}')


def get_key(settings):
    tty.setraw(sys.stdin.fileno())
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def main():
    rclpy.init()
    node     = TeleopRecorder()
    settings = termios.tcgetattr(sys.stdin)
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()

    print("""
+------------------------------------------+
|   drift_asg  TELEOP + RECORDER           |
+------------------------------------------+
|  DRIVE  : wasd   STOP: space             |
|  ARM-L  : rf=shl  tg=elb  yh=wri        |
|  ARM-R  : uj=shl  ik=elb  ol=wri        |
|  GRIPS  : zx=L   cv=R   bn=both         |
|  PRESETS: 1 2 3 4     SPEED: + -        |
+------------------------------------------+
|  [ = start recording                     |
|  ] = stop & save JSON                    |
|  m = manual snapshot                     |
+------------------------------------------+
""")

    try:
        while rclpy.ok():
            key = get_key(settings)
            if key == '\x03':
                break
            handle_key(key, node)
    finally:
        with node._lock:
            still_recording = node._recording
        if still_recording:
            node.stop_recording()
        node.stop_robot()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()