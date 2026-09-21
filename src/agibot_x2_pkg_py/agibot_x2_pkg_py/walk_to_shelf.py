#!/usr/bin/env python3
"""
ROS 2 node that makes the robot walk KINEMATICALLY from the spawn to the work pose in front of the shelf.
The pelvis slides on 4 virtual joints (base_x/y/z + base_yaw, base_controller) while legs_controller animates
non-slipping steps; no balance control, physics stays valid for arms and books. Run it BEFORE pick_test_book.
  ros2 run agibot_x2_pkg_py walk_to_shelf                    # up to WALK_DISTANCE
  ros2 run agibot_x2_pkg_py walk_to_shelf --ros-args -p distance:=0.5 -p arm_swing:=false
  ros2 run agibot_x2_pkg_py walk_to_shelf --ros-args -p dry_run:=true   # trajectory + log only
"""

import math

import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, HistoryPolicy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from sensor_msgs.msg import JointState, Image
from std_msgs.msg import String, Bool
from trajectory_msgs.msg import JointTrajectoryPoint

from agibot_x2_pkg.book_placer import WALK_DISTANCE

BASE_JOINTS = ["base_x_joint", "base_y_joint", "base_z_joint", "base_yaw_joint"]
LEG_JOINTS = {
    "left": ["left_hip_pitch_joint", "left_hip_roll_joint", "left_hip_yaw_joint",
             "left_knee_joint", "left_ankle_pitch_joint", "left_ankle_roll_joint"],
    "right": ["right_hip_pitch_joint", "right_hip_roll_joint", "right_hip_yaw_joint",
              "right_knee_joint", "right_ankle_pitch_joint", "right_ankle_roll_joint"],
}
LEGS_JOINTS = LEG_JOINTS["left"] + LEG_JOINTS["right"]   # legs_controller order
FINGER_MIN = 0.002   # see pick_test_book.FINGER_MIN
ARM_JOINTS = {
    s: [f"{s}_shoulder_pitch_joint", f"{s}_shoulder_roll_joint", f"{s}_shoulder_yaw_joint",
        f"{s}_elbow_joint", f"{s}_wrist_yaw_joint",
        f"{s}_gripper_left_finger_joint", f"{s}_gripper_right_finger_joint"]
    for s in ("left", "right")
}

# sagittal leg geometry (x2_hand_gazebo.urdf)
THIGH = 0.31995   # hip_pitch -> knee (z)
SHANK = 0.282     # knee -> ankle_pitch (z)
FOOT = 0.060      # ankle -> sole
LEG_STRAIGHT = THIGH + SHANK   # 0.602: ankle below the hip with straight leg (sole at 0.662 = spawn height)

# URDF limits (for clamping)
LIM = {
    "hip": (-2.704, 2.556), "knee": (0.0, 2.4073), "ankle": (-0.803, 0.453),
    "shoulder": (-3.08, 2.04), "elbow": (-2.3556, 0.0),
}


def _clamp(v, lim):
    """
    v clamped to (min, max)
    """
    return min(max(v, lim[0]), lim[1])


def _smoothstep(u):
    """
    Smoothstep of u clamped to [0, 1]
    """
    u = min(max(u, 0.0), 1.0)
    return u * u * (3.0 - 2.0 * u)


def leg_ik(x, z):
    """
    Planar 2-link IK: (hip_pitch, knee, ankle_pitch) placing the ankle at (x, z) from the hip, foot level
    x forward, z up (z < 0), knee forward; reach clamped to [|THIGH-SHANK|, THIGH+SHANK].
    URDF signs: positive hip/ankle pitch = foot BACK, knee 0 = straight, positive = flexed.
    """
    r = math.hypot(x, z)
    r = min(max(r, abs(THIGH - SHANK) + 1e-4), LEG_STRAIGHT)   # r = LEG_STRAIGHT -> exactly straight leg
    # knee flexion (0 = straight)
    c = (THIGH ** 2 + SHANK ** 2 - r ** 2) / (2.0 * THIGH * SHANK)
    knee = math.pi - math.acos(min(max(c, -1.0), 1.0))
    # hip->ankle direction from the vertical (positive forward)
    theta = math.atan2(x, -z)
    # angle between thigh and hip->ankle line
    cb = (THIGH ** 2 + r ** 2 - SHANK ** 2) / (2.0 * THIGH * r)
    beta = math.acos(min(max(cb, -1.0), 1.0))
    hip = -(theta + beta)          # positive = thigh back (URDF)
    ankle = -(hip + knee)          # level foot
    return (_clamp(hip, LIM["hip"]), _clamp(knee, LIM["knee"]), _clamp(ankle, LIM["ankle"]))


class WalkToShelf(Node):
    """
    Walk node: plans base, leg and arm-swing trajectories and sends them to the JTCs
    """

    def __init__(self):
        """
        Declare the gait parameters and create the action clients and subscriptions
        """
        super().__init__("walk_to_shelf")
        self.declare_parameter("distance", float(WALK_DISTANCE))  # final base_x_joint [m]
        self.declare_parameter("speed", 0.45)        # m/s (sim time)
        self.declare_parameter("cycle", 1.2)         # s per full cycle (2 steps)
        self.declare_parameter("step_height", 0.04)  # foot lift during swing
        self.declare_parameter("crouch", 0.025)      # base lowering while walking (knees slightly bent)
        self.declare_parameter("clearance", 0.005)   # sole above the floor in stance: no foot contact, no spurious forces
        self.declare_parameter("ramp", 0.6)          # s of acceleration/deceleration
        self.declare_parameter("dt", 0.05)           # waypoint step
        # body motion so the torso does not glide rigidly; all scaled by the envelope (zero at the end)
        self.declare_parameter("bob", 0.02)          # m peak-to-peak, high at mid-stance
        self.declare_parameter("sway", 0.025)        # m amplitude, over the stance foot
        self.declare_parameter("pelvis_yaw", 0.05)   # rad amplitude, towards the advancing leg
        self.declare_parameter("arm_swing", True)
        self.declare_parameter("arm_amplitude", 0.25)  # rad on shoulder pitch
        self.declare_parameter("elbow_flex", 0.35)     # rad of elbow flexion
        self.declare_parameter("dry_run", False)
        # desired camera-to-shelf distance measured with the head_camera depth; if > 0 it replaces `distance`
        # (1.0 m for the library photo, 0.37 for the work pose); head up and torso back keep the optical axis level
        self.declare_parameter("shelf_distance", 0.0)
        self.declare_parameter("look_head_pitch", -0.38)
        self.declare_parameter("look_waist_pitch", -0.31)
        self.declare_parameter("depth_wait_s", 120.0)
        # measure_only: measure and print the shelf distance without walking (to calibrate shelf_distance)
        self.declare_parameter("measure_only", False)

        g = lambda n: self.get_parameter(n).value
        self.distance = float(g("distance"))
        self.shelf_distance = float(g("shelf_distance"))
        self.speed = max(0.05, float(g("speed")))
        self.cycle = max(0.4, float(g("cycle")))
        self.step_height = float(g("step_height"))
        self.crouch = float(g("crouch"))
        self.clearance = float(g("clearance"))
        self.ramp = max(0.1, float(g("ramp")))
        self.dt = max(0.02, float(g("dt")))
        self.bob = float(g("bob"))
        self.sway = float(g("sway"))
        self.pelvis_yaw = float(g("pelvis_yaw"))
        self.arm_swing = bool(g("arm_swing"))
        self.arm_amp = float(g("arm_amplitude"))
        self.elbow_flex = float(g("elbow_flex"))
        self.dry_run = bool(g("dry_run"))
        # stride: distance covered by the body during ONE stance (half cycle)
        self.stride = self.speed * self.cycle / 2.0
        low = self.crouch + self.bob / 2      # lowest pelvis point (double support)
        max_stride = 2.0 * math.sqrt(max(LEG_STRAIGHT ** 2 - (LEG_STRAIGHT - low - self.clearance) ** 2, 1e-6)) * 0.95
        if self.stride > max_stride:
            self.get_logger().warn(
                f"stride {self.stride:.2f} m beyond leg reach ({max_stride:.2f} m): "
                f"reduced (raise crouch or lower speed/cycle)")
            self.stride = max_stride

        self._ac = {
            "base": ActionClient(self, FollowJointTrajectory, "/base_controller/follow_joint_trajectory"),
            "legs": ActionClient(self, FollowJointTrajectory, "/legs_controller/follow_joint_trajectory"),
            "left": ActionClient(self, FollowJointTrajectory, "/left_arm_controller/follow_joint_trajectory"),
            "right": ActionClient(self, FollowJointTrajectory, "/right_arm_controller/follow_joint_trajectory"),
        }
        self.joint_pos = {}
        self.create_subscription(JointState, "/joint_states", self._js_cb, 10)
        self._depth = None
        if self.shelf_distance > 0.0 or bool(self.get_parameter("measure_only").value):
            self._ac["head"] = ActionClient(self, FollowJointTrajectory, "/head_controller/follow_joint_trajectory")
            self._ac["waist"] = ActionClient(self, FollowJointTrajectory, "/waist_controller/follow_joint_trajectory")
            self._trigger = self.create_publisher(Bool, "/head_camera/trigger", 10)
            self.create_subscription(Image, "/head_camera/depth_image", self._depth_cb, 1)
        latched = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.attached = {"left": "", "right": ""}
        for s in ("left", "right"):
            self.create_subscription(String, f"/gripper/{s}/attached",
                                     lambda m, s=s: self.attached.__setitem__(s, m.data), latched)

    def _js_cb(self, msg):
        """
        Store the latest joint positions
        """
        for n, p in zip(msg.name, msg.position):
            self.joint_pos[n] = p

    def _depth_cb(self, msg):
        """
        Store the latest 32FC1 depth image
        """
        if msg.encoding in ("32FC1", ""):
            self._depth = np.frombuffer(msg.data, dtype=np.float32).reshape(msg.height, msg.width).copy()

    def _simple_goal(self, key, names, positions, duration):
        """
        Send a single-waypoint goal (head/waist) and wait for the result
        """
        if not self._ac[key].wait_for_server(timeout_sec=10.0):
            self.get_logger().error(f"{key}: action server not available")
            return False
        fut = self._ac[key].send_goal_async(self._goal(names, [(duration, positions)]))
        rclpy.spin_until_future_complete(self, fut)
        h = fut.result()
        if h is None or not h.accepted:
            return False
        res = h.get_result_async()
        rclpy.spin_until_future_complete(self, res)
        return True

    def measure_shelf_distance(self):
        """
        Distance (m) from the head_camera to the shelf front, with head up and torso back
        Median depth in a central window (25-75 % width, 35-65 % height) between 0.15 and 3 m;
        None if no frame or nothing in front.
        """
        hp, wp = float(self.get_parameter("look_head_pitch").value), float(self.get_parameter("look_waist_pitch").value)
        wy = float(self.joint_pos.get("waist_yaw_joint", 0.0))
        self._simple_goal("head", ["head_yaw_joint", "head_pitch_joint"], [0.0, hp], 1.5)
        self._simple_goal("waist", ["waist_yaw_joint", "waist_pitch_joint"], [wy, wp], 2.0)
        t0 = self.get_clock().now()
        while (self.get_clock().now() - t0).nanoseconds < 1.0e9:
            rclpy.spin_once(self, timeout_sec=0.1)
        d_meas = None
        import time as _time
        for attempt in range(2):                # the first frame of an rgbd sensor can be empty
            self._depth = None
            self._trigger.publish(Bool(data=True))
            t1 = _time.time()
            wait = float(self.get_parameter("depth_wait_s").value)
            while self._depth is None and _time.time() - t1 < wait:
                rclpy.spin_once(self, timeout_sec=0.1)
            if self._depth is None:
                self.get_logger().error("no depth from /head_camera/depth_image (bridge? head_camera sensor?)")
                break
            H, W = self._depth.shape
            win = self._depth[int(0.35 * H):int(0.65 * H), int(0.25 * W):int(0.75 * W)]
            ok = np.isfinite(win) & (win > 0.15) & (win < 3.0)
            if ok.sum() > 200:
                d_meas = float(np.median(win[ok]))
                if d_meas < 0.40:
                    # at the work pose the shelf is at 0.666 m: anything closer than 0.40 m is e.g. an arm in front
                    self.get_logger().error(
                        f"depth: median {d_meas:.3f} m < 0.40: something (a hand?) is in front "
                        "of the camera - invalid measurement, not walking")
                    d_meas = None
                break
            self.get_logger().warn(f"depth: only {int(ok.sum())} valid pixels in the window, retrying")
        self._simple_goal("head", ["head_yaw_joint", "head_pitch_joint"], [0.0, 0.0], 1.5)
        self._simple_goal("waist", ["waist_yaw_joint", "waist_pitch_joint"], [wy, 0.0], 2.0)
        return d_meas

    # gait generation

    def _profile(self, d):
        """
        Trapezoidal path coordinate s(t) from 0 to d > 0; return lists (t, s)
        """
        v, tr = self.speed, self.ramp
        d_ramp = v * tr / 2.0
        if d < 2 * d_ramp:                 # too short to reach v
            tr = math.sqrt(d / v * tr)     # shorter ramps, same mean acceleration
            v = d / tr
            d_ramp = d / 2.0
        t_cruise = (d - 2 * d_ramp) / v
        T = 2 * tr + t_cruise
        ts, ss = [], []
        # times as n*dt, NOT accumulated t += dt: float drift can put a point just before T and the
        # controllers reject the trajectory ("time between points ... not strictly increasing")
        n_last = int(math.floor(T / self.dt - 0.25))   # last index with t <= T - dt/4
        for n in range(1, n_last + 1):
            t = n * self.dt
            if t < tr:
                s = v * t * t / (2 * tr)          # linear velocity ramp
            elif t < tr + t_cruise:
                s = d_ramp + v * (t - tr)
            else:
                td = T - t
                s = d - v * td * td / (2 * tr)
            ts.append(t); ss.append(min(max(s, 0.0), d))
        ts.append(T); ss.append(d)
        return ts, ss

    def _foot(self, phase, env, sign):
        """
        (ankle x relative to the hip, lift) for a leg at `phase`
        phase 0..2pi: [0,pi) swing back->front, [pi,2pi) stance front->back; env in [0,1], sign = walk direction.
        """
        ph = phase % (2 * math.pi)
        S = self.stride * env
        if ph < math.pi:                      # swing
            u = ph / math.pi
            x = -S / 2 + S * _smoothstep(u)
            lift = self.step_height * env * math.sin(math.pi * u)
        else:                                 # stance: linear = foot moves back at body speed, no slipping
            u = (ph - math.pi) / math.pi
            x = S / 2 - S * u
            lift = 0.0
        return sign * x, lift

    def plan(self, x0):
        """
        Waypoints (t, base[4], legs[12], swing_left, swing_right, env) from base_x = x0 to self.distance
        The envelope env starts and ends at 0, so the walk ends with straight legs and the base exactly at
        (distance, 0, 0, 0), where the arm kinematics expects the pelvis (book_placer.ROBOT_SPAWN_X).
        """
        d = self.distance - x0
        sign = 1.0 if d >= 0 else -1.0
        ts, ss = self._profile(abs(d))
        env_len = max(self.stride, 0.15)      # envelope over about one stride
        pts = []
        for t, s in zip(ts, ss):
            env = min(_smoothstep(s / env_len), _smoothstep((abs(d) - s) / env_len))
            phase = math.pi * s / self.stride   # one stance (pi) per stride
            # phase 0/pi = double support (low), pi/2 and 3pi/2 = mid-stance (high)
            base_z = (-self.crouch - self.bob / 2 * math.cos(2 * phase)) * env
            # left leg in stance for phase in [pi, 2pi): body towards +y
            base_y = -self.sway * math.sin(phase) * env
            # pelvis turned towards the advancing leg (left swinging = +yaw)
            base_yaw = self.pelvis_yaw * math.sin(phase) * env
            legs = []
            feet = {}
            for side, ph in (("left", phase), ("right", phase + math.pi)):
                xf, lift = self._foot(ph, env, sign)
                feet[side] = xf
                # ankle relative to the hip: sole at `clearance*env` above the floor
                z_ankle = -(LEG_STRAIGHT + base_z) + self.clearance * env + lift
                hip, knee, ankle = leg_ik(xf, z_ankle)
                legs += [hip, 0.0, 0.0, knee, ankle, 0.0]
            base = [x0 + sign * s, base_y, base_z, base_yaw]
            half = max(self.stride / 2, 1e-3)
            # contralateral: right arm forward (negative pitch) with left foot forward
            sw_r = -self.arm_amp * feet["left"] / half * env
            sw_l = -self.arm_amp * feet["right"] / half * env
            pts.append((t, base, legs, sw_l, sw_r, env))
        return pts

    # sending

    @staticmethod
    def _goal(names, wps):
        """
        FollowJointTrajectory goal from [(t, positions)]
        """
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(names)
        for t, q in wps:
            pt = JointTrajectoryPoint()
            pt.positions = [float(v) for v in q]
            ns = int(round(t * 1e9))
            pt.time_from_start = Duration(sec=ns // 1_000_000_000, nanosec=ns % 1_000_000_000)
            goal.trajectory.points.append(pt)
        return goal

    def run(self):
        """
        Wait for joint states, optionally measure the shelf distance, plan and execute the walk
        Arms holding an object (/gripper/<side>/attached not empty) do not swing.
        """
        # current state (base_x and arms needed)
        needed = BASE_JOINTS + LEGS_JOINTS + ARM_JOINTS["left"] + ARM_JOINTS["right"]
        t0 = self.get_clock().now()
        while any(n not in self.joint_pos for n in needed):
            rclpy.spin_once(self, timeout_sec=0.2)
            if (self.get_clock().now() - t0).nanoseconds > 15e9:
                missing = [n for n in needed if n not in self.joint_pos]
                self.get_logger().error(f"/joint_states without {missing[:4]}...: old URDF/controllers? "
                                        "Rebuild agibot_x2_pkg and relaunch.")
                return False
        x0 = self.joint_pos["base_x_joint"]
        if bool(self.get_parameter("measure_only").value):
            d_meas = self.measure_shelf_distance()
            if d_meas is None:
                return False
            self.get_logger().info(
                f"MEASURE: shelf at {d_meas:.3f} m from the camera (base_x {x0:.3f}); no walking")
            return True
        if self.shelf_distance > 0.0 and not self.dry_run:
            d_meas = self.measure_shelf_distance()
            if d_meas is None:
                self.get_logger().error("shelf distance not measurable: not walking")
                return False
            self.distance = x0 + (d_meas - self.shelf_distance)
            self.get_logger().info(
                f"Shelf at {d_meas:.3f} m from the camera (depth measurement), wanted {self.shelf_distance:.2f}: "
                f"walking {d_meas - self.shelf_distance:+.3f} m -> base_x {self.distance:.3f}")
        if abs(self.distance - x0) < 0.005:
            self.get_logger().info(f"base_x already at {x0:.3f} m: nothing to do")
            return True

        pts = self.plan(x0)
        T = pts[-1][0]
        n_steps = int(round(abs(self.distance - x0) / self.stride))
        self.get_logger().info(
            f"Walk: base_x {x0:.2f} -> {self.distance:.2f} m, {self.speed:.2f} m/s, "
            f"stride {self.stride:.2f} m (~{n_steps} steps), {T:.1f} s sim, {len(pts)} waypoints, "
            f"arms: {'oscillano' if self.arm_swing else 'ferme'}")
        legs_max = np.max(np.abs([p[2] for p in pts]), axis=0)
        self.get_logger().info(
            f"legs max |hip| {legs_max[0]:.2f} |knee| {legs_max[3]:.2f} "
            f"|ankle| {legs_max[4]:.2f} rad; base_z min {min(p[1][2] for p in pts):.3f} m")

        goals = {
            "base": self._goal(BASE_JOINTS, [(p[0], p[1]) for p in pts]),
            "legs": self._goal(LEGS_JOINTS, [(p[0], p[2]) for p in pts]),
        }
        if self.arm_swing:
            for side, idx in (("left", 3), ("right", 4)):
                if self.attached.get(side):
                    self.get_logger().info(
                        f"arm {side}: holding '{self.attached[side]}', not swinging")
                    continue
                q_now = [self.joint_pos[n] for n in ARM_JOINTS[side]]
                # fingers stay put but NEVER at 0.0 (end stop): outer fingers driven to 0 stay stuck forever
                # (Bugs.md), and /joint_states reports exactly 0.0 at startup
                q_now[5] = max(q_now[5], FINGER_MIN)
                q_now[6] = max(q_now[6], FINGER_MIN)
                wps = []
                for p in pts:
                    q = list(q_now)
                    q[0] = _clamp(q_now[0] + p[idx], LIM["shoulder"])
                    q[3] = _clamp(q_now[3] - self.elbow_flex * p[5], LIM["elbow"])
                    wps.append((p[0], q))
                goals[side] = self._goal(ARM_JOINTS[side], wps)

        if self.dry_run:
            self.get_logger().info("dry_run: no goal sent")
            return True
        for k in goals:
            if not self._ac[k].wait_for_server(timeout_sec=10.0):
                self.get_logger().error(f"action server {k} not available "
                                        "(base_controller/legs_controller active?)")
                return False
        futs = {k: self._ac[k].send_goal_async(g) for k, g in goals.items()}
        for f in futs.values():
            rclpy.spin_until_future_complete(self, f)
        handles = {k: f.result() for k, f in futs.items()}
        bad = [k for k, h in handles.items() if h is None or not h.accepted]
        if bad:
            self.get_logger().error(f"goal rejected: {bad}")
            return False
        for k, h in handles.items():
            r = h.get_result_async()
            rclpy.spin_until_future_complete(self, r)
        rclpy.spin_once(self, timeout_sec=0.5)
        xf = self.joint_pos.get("base_x_joint", float("nan"))
        self.get_logger().info(f"Arrived: base_x = {xf:.3f} m (work pose: pelvis at ROBOT_SPAWN_X). "
                               "Next: ros2 run agibot_x2_pkg_py pick_test_book --ros-args -p book:=it")
        return True


def main(args=None):
    """
    Run the walk once and exit with 0 on success
    """
    rclpy.init(args=args)
    node = WalkToShelf()
    ok = False
    try:
        ok = node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    raise SystemExit(0 if ok else 1)


if __name__ == "__main__":
    main()
