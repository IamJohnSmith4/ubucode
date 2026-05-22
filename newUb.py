#!/usr/bin/env python3
"""
Robot Navigation with Human Obstacle Detection & Dodge
=======================================================
Uses LiDAR (/scan) to detect humans (or any obstacle) in front of
the robot and automatically dodge left/right while navigating.

Detection zones (configurable at top of file):
  - FRONT zone  : obstacle triggers dodge
  - SIDE zones  : used to decide which way to dodge
  - CLEAR zone  : robot resumes normal navigation

Dodge logic:
  1. Obstacle detected → pause move_base goal
  2. Check left / right side for free space
  3. Strafe / arc toward the free side
  4. Re-check front until clear
  5. Resume original move_base goal
"""

import rospy
import math
import signal
import sys
import threading
import time

from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
from tf.transformations import euler_from_quaternion
from flask import Flask, request, jsonify
import actionlib

# ==================================================
# GLOBAL SETTINGS & STATE
# ==================================================
is_navigating      = False
current_location   = 1
current_progress   = 0
velocity_publisher = None

# ==================================================
# OBSTACLE / DODGE SETTINGS  (tune these)
# ==================================================

# Front detection cone: ±FRONT_HALF_DEG degrees from robot heading
FRONT_HALF_DEG   = 30          # degrees either side of forward

# Distance thresholds (metres)
OBSTACLE_DIST    = 0.8         # obstacle closer than this → start dodge
CLEAR_DIST       = 1.2         # obstacle must be farther than this → resume
SIDE_CHECK_DIST  = 0.6         # if side is blocked closer than this → not safe

# Dodge speed  (m/s and rad/s)
DODGE_LINEAR_X   = 0.15        # forward creep while dodging
DODGE_ANGULAR_Z  = 0.45        # rotation during dodge

# How long (s) to keep rotating during one dodge step
DODGE_STEP_DUR   = 0.6

# How many times to retry dodge before giving up
MAX_DODGE_RETRY  = 8

# ==================================================
# NODE MAP
# ==================================================
NODE_POSES = {
    1: {"x":  3.516, "y":  -0.651, "yaw_rad":  0.0},
    2: {"x": 12.455, "y":  13.314, "yaw_rad":  0.0},
    3: {"x": 16.521, "y":  22.191, "yaw_rad":  0.0},
}


def signal_handler(sig, frame):
    print("\n[INFO] Ctrl+C — Stopping Robot.")
    if velocity_publisher:
        velocity_publisher.publish(Twist())
    rospy.signal_shutdown("User Interrupted")
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)


# ==================================================
# PID CONTROLLER
# ==================================================
class PID:
    def __init__(self, kp, ki, kd, min_val, max_val):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.min_val, self.max_val = min_val, max_val
        self.integral, self.last_error = 0.0, 0.0

    def compute(self, error, dt):
        self.integral  += error * dt
        derivative      = (error - self.last_error) / dt
        output          = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)
        self.last_error = error
        return max(min(output, self.max_val), self.min_val)


# ==================================================
# LASER SCAN ANALYSER
# ==================================================
class ScanAnalyser:
    """
    Subscribes to /scan and answers:
      - is_obstacle_front()  → bool
      - is_side_clear(side)  → bool  side='left'|'right'
      - min_front_dist()     → float (metres)
    """

    def __init__(self):
        self._lock      = threading.Lock()
        self._ranges    = []
        self._angle_min = 0.0
        self._angle_inc = 0.0
        self._ready     = False

        rospy.Subscriber("/scan", LaserScan, self._cb, queue_size=1)
        rospy.loginfo("[Scan] Waiting for /scan …")
        rospy.wait_for_message("/scan", LaserScan, timeout=10.0)
        rospy.loginfo("[Scan] Ready.")

    def _cb(self, msg):
        with self._lock:
            self._ranges    = list(msg.ranges)
            self._angle_min = msg.angle_min
            self._angle_inc = msg.angle_increment
            self._ready     = True

    def _get_sector_min(self, angle_from_deg, angle_to_deg):
        """Return minimum range in a sector (degrees, robot-frame)."""
        with self._lock:
            if not self._ready or not self._ranges:
                return float('inf')
            r      = list(self._ranges)
            a_min  = self._angle_min
            a_inc  = self._angle_inc

        af = math.radians(angle_from_deg)
        at = math.radians(angle_to_deg)

        values = []
        for i, v in enumerate(r):
            angle = a_min + i * a_inc
            if af <= angle <= at:
                if not math.isnan(v) and not math.isinf(v) and v > 0.05:
                    values.append(v)

        return min(values) if values else float('inf')

    def min_front_dist(self):
        return self._get_sector_min(-FRONT_HALF_DEG, FRONT_HALF_DEG)

    def is_obstacle_front(self):
        return self.min_front_dist() < OBSTACLE_DIST

    def is_front_clear(self):
        return self.min_front_dist() > CLEAR_DIST

    def left_min_dist(self):
        # 45°→135° (left side of robot)
        return self._get_sector_min(45, 135)

    def right_min_dist(self):
        # -135°→-45° (right side of robot)
        return self._get_sector_min(-135, -45)

    def best_dodge_side(self):
        """
        Returns 'left', 'right', or 'back' based on which side has more free space.
        """
        ld = self.left_min_dist()
        rd = self.right_min_dist()
        rospy.loginfo(f"[Dodge] left={ld:.2f}m  right={rd:.2f}m")

        if ld > SIDE_CHECK_DIST and ld >= rd:
            return 'left'
        elif rd > SIDE_CHECK_DIST and rd > ld:
            return 'right'
        else:
            return 'back'   # both sides blocked → back up a little


# ==================================================
# ROBOT CONTROL CLASS
# ==================================================
class OdomRobot:
    def __init__(self, pub):
        global is_navigating
        rospy.init_node("odom_robot")
        self.pub = pub

        # --- Odometry ---
        self.raw_x, self.raw_y, self.raw_yaw = 0.0, 0.0, 0.0
        self.x, self.y, self.yaw             = 0.0, 0.0, 0.0
        self.offset_x, self.offset_y, self.offset_yaw = 0.0, 0.0, 0.0

        # --- AMCL ---
        self.amcl_x, self.amcl_y, self.amcl_yaw = 0.0, 0.0, 0.0
        self.amcl_covariance = 1.0
        self.amcl_ready      = False
        self.amcl_lock       = threading.Lock()

        # --- Laser scan analyser ---
        self.scan = ScanAnalyser()

        # --- Dodge state ---
        self._dodging          = False
        self._dodge_lock       = threading.Lock()
        self.last_obstacle_at  = None   # 'front' | None
        self.obstacle_status   = "clear"  # human-readable for /status

        # --- move_base ---
        rospy.loginfo("[move_base] Connecting …")
        self.move_base_client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        mb_ok = self.move_base_client.wait_for_server(timeout=rospy.Duration(10.0))
        rospy.loginfo("[move_base] Connected!" if mb_ok else "[move_base] NOT available.")

        # --- Subscribers ---
        rospy.Subscriber("/odom",      Odometry,                  self.odom_callback)
        rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.amcl_callback)

        self.pid_straight = PID(kp=1.8, ki=0.005, kd=0.1, min_val=-0.4, max_val=0.4)
        self.pid_rotate   = PID(kp=1.0, ki=0.01,  kd=0.1, min_val=-0.3, max_val=0.3)

        rospy.loginfo("Waiting for odom …")
        rospy.wait_for_message("/odom", Odometry)
        rospy.sleep(1)

        self._wait_for_amcl(timeout=10.0)
        self._set_amcl_to_node(1)
        rospy.loginfo("=== READY: Start at Node 1 ===")

    # --------------------------------------------------
    # CALLBACKS
    # --------------------------------------------------
    def odom_callback(self, msg):
        self.raw_x = msg.pose.pose.position.x
        self.raw_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        (_, _, yaw) = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.raw_yaw = yaw
        diff_yaw = self.raw_yaw - self.offset_yaw
        self.x   = self.raw_x - self.offset_x
        self.y   = self.raw_y - self.offset_y
        self.yaw = math.atan2(math.sin(diff_yaw), math.cos(diff_yaw))

    def amcl_callback(self, msg):
        with self.amcl_lock:
            self.amcl_x = msg.pose.pose.position.x
            self.amcl_y = msg.pose.pose.position.y
            q = msg.pose.pose.orientation
            (_, _, self.amcl_yaw) = euler_from_quaternion([q.x, q.y, q.z, q.w])
            cov = msg.pose.covariance
            self.amcl_covariance = (cov[0] + cov[7]) / 2.0
            if self.amcl_covariance < 0.05:
                self.amcl_ready = True

    # --------------------------------------------------
    # AMCL HELPERS
    # --------------------------------------------------
    def _wait_for_amcl(self, timeout=10.0):
        rospy.loginfo("Waiting for /amcl_pose (%.0fs) …" % timeout)
        try:
            rospy.wait_for_message("/amcl_pose", PoseWithCovarianceStamped, timeout=timeout)
            rospy.loginfo("[AMCL] Running.")
        except rospy.ROSException:
            rospy.logwarn("[AMCL] Not received. Odometry only.")

    def set_initial_pose(self, x, y, yaw_rad):
        pub = rospy.Publisher("/initialpose", PoseWithCovarianceStamped,
                              queue_size=1, latch=True)
        rospy.sleep(0.5)
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = "map"
        msg.header.stamp    = rospy.Time.now()
        msg.pose.pose.position.x    = x
        msg.pose.pose.position.y    = y
        msg.pose.pose.orientation.z = math.sin(yaw_rad / 2.0)
        msg.pose.pose.orientation.w = math.cos(yaw_rad / 2.0)
        cov = [0.0] * 36
        cov[0]  = 0.25
        cov[7]  = 0.25
        cov[35] = 0.068
        msg.pose.covariance = cov
        pub.publish(msg)
        rospy.loginfo(f"[AMCL] Pose → x={x:.2f} y={y:.2f} yaw={math.degrees(yaw_rad):.1f}°")

    def _set_amcl_to_node(self, node_id):
        if node_id not in NODE_POSES:
            return
        p = NODE_POSES[node_id]
        self.set_initial_pose(p["x"], p["y"], p["yaw_rad"])
        rospy.loginfo(f"[AMCL] Reset to Node {node_id}")

    def get_best_pose(self):
        with self.amcl_lock:
            if self.amcl_ready and self.amcl_covariance < 0.05:
                return self.amcl_x, self.amcl_y, self.amcl_yaw, "amcl"
        return self.x, self.y, self.yaw, "odom"

    # --------------------------------------------------
    # RESET HOME
    # --------------------------------------------------
    def execute_home_sequence(self):
        global is_navigating, current_location
        rospy.loginfo("--- Reset to Node 1 ---")
        is_navigating    = False
        current_location = 1
        self.move_base_client.cancel_all_goals()
        self._set_amcl_to_node(1)
        rospy.loginfo("--- Reset Done: Node=1 ---")

    # --------------------------------------------------
    # LOW-LEVEL CMD_VEL HELPERS
    # --------------------------------------------------
    def _send_vel(self, linear_x=0.0, angular_z=0.0, duration=0.0):
        """Publish a Twist for `duration` seconds (blocking)."""
        cmd = Twist()
        cmd.linear.x  = linear_x
        cmd.angular.z = angular_z
        if duration <= 0:
            self.pub.publish(cmd)
            return
        end = time.time() + duration
        rate = rospy.Rate(20)
        while time.time() < end and not rospy.is_shutdown():
            self.pub.publish(cmd)
            rate.sleep()

    def _stop(self):
        self.pub.publish(Twist())

    # --------------------------------------------------
    # DODGE BEHAVIOUR
    # --------------------------------------------------
    def _dodge_obstacle(self):
        """
        Called when an obstacle is detected in the front zone.
        Pauses move_base, manoeuvres around the obstacle,
        then re-sends the same goal so move_base re-plans.

        Returns True if the front is now clear, False if gave up.
        """
        with self._dodge_lock:
            if self._dodging:
                return False
            self._dodging = True

        try:
            rospy.logwarn("[Dodge] Obstacle detected! Cancelling move_base goal …")
            self.obstacle_status = "blocked"
            self.move_base_client.cancel_all_goals()
            self._stop()
            rospy.sleep(0.3)

            for attempt in range(1, MAX_DODGE_RETRY + 1):
                if not is_navigating:
                    rospy.loginfo("[Dodge] Navigation cancelled externally.")
                    return False

                if self.scan.is_front_clear():
                    rospy.loginfo(f"[Dodge] Front clear after {attempt-1} steps.")
                    self.obstacle_status = "clear"
                    return True

                side = self.scan.best_dodge_side()
                dist = self.scan.min_front_dist()
                rospy.logwarn(
                    f"[Dodge] Attempt {attempt}/{MAX_DODGE_RETRY} | "
                    f"front={dist:.2f}m | dodge→{side}"
                )

                if side == 'left':
                    # Arc left: rotate CCW while creeping forward
                    self._send_vel(
                        linear_x=DODGE_LINEAR_X,
                        angular_z=+DODGE_ANGULAR_Z,
                        duration=DODGE_STEP_DUR
                    )
                elif side == 'right':
                    # Arc right: rotate CW while creeping forward
                    self._send_vel(
                        linear_x=DODGE_LINEAR_X,
                        angular_z=-DODGE_ANGULAR_Z,
                        duration=DODGE_STEP_DUR
                    )
                else:
                    # Both sides blocked → back up slowly
                    rospy.logwarn("[Dodge] Both sides blocked — backing up!")
                    self._send_vel(
                        linear_x=-0.15,
                        angular_z=0.0,
                        duration=0.8
                    )

                self._stop()
                rospy.sleep(0.2)

            rospy.logerr("[Dodge] Gave up after max retries.")
            self.obstacle_status = "stuck"
            return False

        finally:
            self._dodging = False

    # --------------------------------------------------
    # move_base NAVIGATION  (with obstacle watch-loop)
    # --------------------------------------------------
    def _build_goal(self, node_id):
        if node_id not in NODE_POSES:
            rospy.logerr(f"Node {node_id} not in NODE_POSES")
            return None
        p    = NODE_POSES[node_id]
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp    = rospy.Time.now()
        goal.target_pose.pose.position.x = p["x"]
        goal.target_pose.pose.position.y = p["y"]
        yr = p["yaw_rad"]
        goal.target_pose.pose.orientation.z = math.sin(yr / 2.0)
        goal.target_pose.pose.orientation.w = math.cos(yr / 2.0)
        return goal

    def navigate_to_node(self, target_node):
        global is_navigating, current_progress

        goal = self._build_goal(target_node)
        if goal is None:
            return False

        tx = NODE_POSES[target_node]["x"]
        ty = NODE_POSES[target_node]["y"]
        rospy.loginfo(f"[Nav] → Node {target_node} ({tx:.2f}, {ty:.2f})")

        self.move_base_client.send_goal(goal)
        current_progress     = 0
        self.obstacle_status = "clear"
        rate = rospy.Rate(5)   # 5 Hz check loop

        while not rospy.is_shutdown():

            # ── External cancel ──────────────────────────────────────────
            if not is_navigating:
                self.move_base_client.cancel_goal()
                rospy.loginfo("[Nav] Goal cancelled externally.")
                self._stop()
                return False

            # ── Obstacle check ───────────────────────────────────────────
            if self.scan.is_obstacle_front() and not self._dodging:
                dist = self.scan.min_front_dist()
                rospy.logwarn(
                    f"[Nav] ⚠ Obstacle at {dist:.2f}m — starting dodge …"
                )
                # Run dodge in same thread (blocks until clear or gave up)
                cleared = self._dodge_obstacle()

                if not is_navigating:
                    return False

                if cleared:
                    # Re-send the original goal so move_base re-plans
                    rospy.loginfo("[Nav] Obstacle cleared — re-sending goal …")
                    goal.target_pose.header.stamp = rospy.Time.now()
                    self.move_base_client.send_goal(goal)
                    self.obstacle_status = "clear"
                else:
                    rospy.logerr("[Nav] Could not clear obstacle. Aborting.")
                    self._stop()
                    return False

            # ── Goal state check ─────────────────────────────────────────
            state = self.move_base_client.get_state()

            if state == GoalStatus.SUCCEEDED:
                current_progress     = 100
                self.obstacle_status = "clear"
                rospy.loginfo(f"[Nav] ✓ Reached Node {target_node}!")
                return True

            elif state in (GoalStatus.ABORTED, GoalStatus.REJECTED,
                           GoalStatus.PREEMPTED, GoalStatus.LOST):
                if self._dodging:
                    rate.sleep()
                    continue
                rospy.logwarn(f"[Nav] move_base failed. State={state}")
                return False

            # ── Progress update ──────────────────────────────────────────
            bx, by, _, _ = self.get_best_pose()
            dist_rem = math.sqrt((tx - bx)**2 + (ty - by)**2)
            src      = NODE_POSES.get(current_location, {"x": bx, "y": by})
            dist_tot = math.sqrt((tx - src["x"])**2 + (ty - src["y"])**2)
            if dist_tot > 0.01:
                current_progress = max(0, min(99,
                    (1 - dist_rem / dist_tot) * 100))

            rospy.loginfo(
                f"[Nav] progress={current_progress:.1f}%  "
                f"dist={dist_rem:.2f}m  "
                f"front={self.scan.min_front_dist():.2f}m  "
                f"obstacle={self.obstacle_status}"
            )
            rate.sleep()

        return False

    def execute_path(self, start, target):
        return self.navigate_to_node(target)


# ==================================================
# FLASK API SERVER
# ==================================================
app      = Flask(__name__)
my_robot = None


@app.route('/command', methods=['POST'])
def handle_command():
    global is_navigating, current_progress
    data   = request.json or {}
    start  = data.get('start')
    target = data.get('target')

    if is_navigating:
        return jsonify({"status": "error", "message": "Robot is busy"}), 400
    if target not in NODE_POSES:
        return jsonify({"status": "error",
                        "message": f"Node {target} not in NODE_POSES"}), 400

    def run_and_finish(s, t):
        global is_navigating, current_location, current_progress
        current_progress = 0
        is_navigating    = True
        success = my_robot.execute_path(s, t)
        is_navigating = False
        if success:
            current_location = t

    threading.Thread(target=run_and_finish, args=(start, target)).start()
    return jsonify({"status": "starting",
                    "target_node": target,
                    "target_pose": NODE_POSES[target]}), 200


@app.route('/status', methods=['GET'])
def get_status():
    bx, by, byaw, src = my_robot.get_best_pose()
    return jsonify({
        "is_navigating":    is_navigating,
        "current_location": current_location,
        "current_progress": round(current_progress, 1),
        "odom_position":    {"x": round(my_robot.x, 3), "y": round(my_robot.y, 3)},
        "odom_yaw_deg":     round(math.degrees(my_robot.yaw), 2),
        "amcl_position":    {"x": round(my_robot.amcl_x, 3), "y": round(my_robot.amcl_y, 3)},
        "amcl_yaw_deg":     round(math.degrees(my_robot.amcl_yaw), 2),
        "amcl_covariance":  round(my_robot.amcl_covariance, 4),
        "amcl_ready":       my_robot.amcl_ready,
        "best_position":    {"x": round(bx, 3), "y": round(by, 3)},
        "best_yaw_deg":     round(math.degrees(byaw), 2),
        "pose_source":      src,
        # --- NEW: obstacle info ---
        "obstacle_status":  my_robot.obstacle_status,
        "front_dist_m":     round(my_robot.scan.min_front_dist(), 3),
        "is_dodging":       my_robot._dodging,
    })


@app.route('/stop', methods=['POST', 'GET'])
def stop_robot():
    global is_navigating
    is_navigating = False
    my_robot.move_base_client.cancel_all_goals()
    velocity_publisher.publish(Twist())
    return jsonify({"status": "success", "message": "Stopped"}), 200


@app.route('/command/reset-home', methods=['POST'])
def handle_reset_home():
    global is_navigating
    is_navigating = False
    my_robot.move_base_client.cancel_all_goals()
    threading.Thread(target=my_robot.execute_home_sequence).start()
    return jsonify({"status": "success",
                    "message": "Reset to Node 1"}), 200


@app.route('/amcl/set-pose', methods=['POST'])
def handle_set_pose():
    data    = request.json or {}
    x       = float(data.get("x",       0.0))
    y       = float(data.get("y",       0.0))
    yaw_deg = float(data.get("yaw_deg", 0.0))
    yaw_rad = math.radians(yaw_deg)
    threading.Thread(target=my_robot.set_initial_pose, args=(x, y, yaw_rad)).start()
    return jsonify({"status": "success",
                    "message": f"Pose → x={x} y={y} yaw={yaw_deg}°"}), 200


@app.route('/amcl/status', methods=['GET'])
def handle_amcl_status():
    return jsonify({
        "amcl_ready":      my_robot.amcl_ready,
        "amcl_covariance": round(my_robot.amcl_covariance, 4),
        "amcl_position":   {"x": round(my_robot.amcl_x, 3),
                            "y": round(my_robot.amcl_y, 3)},
        "amcl_yaw_deg":    round(math.degrees(my_robot.amcl_yaw), 2),
        "note":            "covariance < 0.05 = confident"
    })


@app.route('/nodes', methods=['GET'])
def get_nodes():
    out = {k: {**v, "yaw_deg": round(math.degrees(v["yaw_rad"]), 2)}
           for k, v in NODE_POSES.items()}
    return jsonify({"nodes": out})


@app.route('/nodes/<int:node_id>', methods=['POST'])
def update_node(node_id):
    data = request.json or {}
    NODE_POSES[node_id] = {
        "x":       float(data.get("x",       0.0)),
        "y":       float(data.get("y",       0.0)),
        "yaw_rad": math.radians(float(data.get("yaw_deg", 0.0))),
    }
    return jsonify({"status": "updated", "node": node_id,
                    "pose": NODE_POSES[node_id]}), 200


# --- NEW: live obstacle scan endpoint ---
@app.route('/obstacle', methods=['GET'])
def get_obstacle():
    return jsonify({
        "obstacle_in_front": my_robot.scan.is_obstacle_front(),
        "front_dist_m":      round(my_robot.scan.min_front_dist(), 3),
        "left_dist_m":       round(my_robot.scan.left_min_dist(), 3),
        "right_dist_m":      round(my_robot.scan.right_min_dist(), 3),
        "best_dodge_side":   my_robot.scan.best_dodge_side(),
        "is_dodging":        my_robot._dodging,
        "obstacle_status":   my_robot.obstacle_status,
        "thresholds": {
            "obstacle_dist_m":   OBSTACLE_DIST,
            "clear_dist_m":      CLEAR_DIST,
            "side_check_dist_m": SIDE_CHECK_DIST,
            "front_cone_deg":    FRONT_HALF_DEG * 2,
        }
    })


# --- NEW: update dodge settings at runtime ---
@app.route('/obstacle/settings', methods=['POST'])
def update_obstacle_settings():
    global OBSTACLE_DIST, CLEAR_DIST, SIDE_CHECK_DIST, FRONT_HALF_DEG
    global DODGE_LINEAR_X, DODGE_ANGULAR_Z, DODGE_STEP_DUR
    data = request.json or {}
    if "obstacle_dist"    in data: OBSTACLE_DIST    = float(data["obstacle_dist"])
    if "clear_dist"       in data: CLEAR_DIST       = float(data["clear_dist"])
    if "side_check_dist"  in data: SIDE_CHECK_DIST  = float(data["side_check_dist"])
    if "front_half_deg"   in data: FRONT_HALF_DEG   = float(data["front_half_deg"])
    if "dodge_linear_x"   in data: DODGE_LINEAR_X   = float(data["dodge_linear_x"])
    if "dodge_angular_z"  in data: DODGE_ANGULAR_Z  = float(data["dodge_angular_z"])
    if "dodge_step_dur"   in data: DODGE_STEP_DUR   = float(data["dodge_step_dur"])
    return jsonify({
        "status": "updated",
        "obstacle_dist":   OBSTACLE_DIST,
        "clear_dist":      CLEAR_DIST,
        "side_check_dist": SIDE_CHECK_DIST,
        "front_half_deg":  FRONT_HALF_DEG,
        "dodge_linear_x":  DODGE_LINEAR_X,
        "dodge_angular_z": DODGE_ANGULAR_Z,
        "dodge_step_dur":  DODGE_STEP_DUR,
    })


# ==================================================
# MAIN
# ==================================================
if __name__ == "__main__":
    velocity_publisher = rospy.Publisher(
        '/mobile_base/commands/velocity', Twist, queue_size=10)

    my_robot = OdomRobot(pub=velocity_publisher)

    current_location = 1
    is_navigating    = False

    print("--- Robot Server Ready on Port 5000 ---")
    print("  POST /command                    {start, target}")
    print("  GET  /status                     (includes obstacle info)")
    print("  POST /stop")
    print("  POST /command/reset-home         → Reset AMCL to Node 1")
    print("  POST /amcl/set-pose              {x, y, yaw_deg}")
    print("  GET  /amcl/status")
    print("  GET  /nodes")
    print("  POST /nodes/<id>                 {x, y, yaw_deg}")
    print("  GET  /obstacle                   live scan data")
    print("  POST /obstacle/settings          tune dodge params at runtime")
    app.run(host='0.0.0.0', port=5000, debug=False, threaded=True)
