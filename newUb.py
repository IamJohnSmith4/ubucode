#!/usr/bin/env python3
import rospy
import math
import signal
import sys
import threading
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped, PoseStamped
from nav_msgs.msg import Odometry
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
# NODE MAP
# ==================================================
NODE_POSES = {
    0: {"x":  9.777, "y":  -0.265, "yaw_rad":  0.000},   # ← Hub waypoint
    1: {"x":  2.379, "y":  -0.639, "yaw_rad":  0.012},
    2: {"x": 12.455, "y":  13.314, "yaw_rad":  0.002},
    3: {"x": 16.521, "y":  22.191, "yaw_rad":  0.002},
}

# ==================================================
# ROUTING TABLE
# Node 0 is a mandatory hub between Node 1 and Nodes 2/3.
#
#   Node 1  → (2 or 3) : must pass Node 0 first  → [0, target]
#   Node 2/3 → Node 1   : must pass Node 0 first  → [0, 1]
#   Node 2  → Node 3    : direct (both on same side of hub)
#   Node 3  → Node 2    : direct
#   Any     → Node 0    : direct
#   Node 0  → Any       : direct
# ==================================================
def get_waypoints(start: int, target: int) -> list[int]:
    """
    Returns an ordered list of node IDs to visit (excluding start),
    routing through Node 0 when crossing between the Node-1 side
    and the Node-2/3 side.
    """
    if start == target:
        return []

    side_A = {1}          # "home" side
    side_B = {2, 3}       # "far" side
    HUB    = 0

    # Direct trips that already involve the hub
    if start == HUB or target == HUB:
        return [target]

    # Crossing sides → insert hub
    if (start in side_A and target in side_B) or \
       (start in side_B and target in side_A):
        return [HUB, target]

    # Same side (e.g. 2 → 3 or 3 → 2) → direct
    return [target]


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

        # --- move_base ---
        rospy.loginfo("[move_base] Connecting...")
        self.move_base_client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        mb_ok = self.move_base_client.wait_for_server(timeout=rospy.Duration(10.0))
        rospy.loginfo("[move_base] Connected!" if mb_ok else "[move_base] NOT available.")

        # --- Subscribers ---
        rospy.Subscriber("/odom",      Odometry,                  self.odom_callback)
        rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.amcl_callback)

        self.pid_straight = PID(kp=1.8, ki=0.005, kd=0.1, min_val=-0.4, max_val=0.4)
        self.pid_rotate   = PID(kp=1.0, ki=0.01,  kd=0.1, min_val=-0.3, max_val=0.3)

        rospy.loginfo("Waiting for odom...")
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
        rospy.loginfo("Waiting for /amcl_pose (%.0fs)..." % timeout)
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
    # RESET HOME  ✅ ไม่ขยับหุ่น แค่ reset AMCL = Node 1
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
    # move_base NAVIGATION
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

    def navigate_to_node(self, target_node, total_legs, leg_index):
        """
        Navigate to a single node. Progress is scaled across all legs so
        the reported percentage reflects the full multi-waypoint journey.

        total_legs : total number of waypoint hops in this trip
        leg_index  : 0-based index of this hop
        """
        global is_navigating, current_progress

        goal = self._build_goal(target_node)
        if goal is None:
            return False

        tx = NODE_POSES[target_node]["x"]
        ty = NODE_POSES[target_node]["y"]
        rospy.loginfo(f"[Nav] → Node {target_node} ({tx:.2f}, {ty:.2f})  "
                      f"[leg {leg_index+1}/{total_legs}]")

        self.move_base_client.send_goal(goal)
        rate = rospy.Rate(2)

        while not rospy.is_shutdown():

            if not is_navigating:
                self.move_base_client.cancel_goal()
                rospy.loginfo("[Nav] Goal cancelled.")
                return False

            state = self.move_base_client.get_state()

            if state == GoalStatus.SUCCEEDED:
                # Mark this leg as fully done
                current_progress = round((leg_index + 1) / total_legs * 100)
                rospy.loginfo(f"[Nav] ✓ Reached Node {target_node}!")
                return True

            elif state in (GoalStatus.ABORTED, GoalStatus.REJECTED,
                           GoalStatus.PREEMPTED, GoalStatus.LOST):
                rospy.logwarn(f"[Nav] Failed. State={state}")
                return False

            # Estimate progress within this leg
            bx, by, _, _ = self.get_best_pose()
            dist_rem = math.sqrt((tx - bx)**2 + (ty - by)**2)
            src = NODE_POSES.get(current_location, {"x": bx, "y": by})
            dist_tot = math.sqrt((tx - src["x"])**2 + (ty - src["y"])**2)

            leg_pct = 0.0
            if dist_tot > 0.01:
                leg_pct = max(0.0, min(1.0, 1.0 - dist_rem / dist_tot))

            # Scale into overall progress
            overall = (leg_index + leg_pct) / total_legs * 100
            current_progress = round(max(current_progress, overall), 1)

            rospy.loginfo(f"[Nav] progress={current_progress:.1f}% dist={dist_rem:.2f}m")
            rate.sleep()

        return False

    def execute_path(self, start, target):
        """
        Build the waypoint list from the routing table and execute each hop
        in sequence. Updates current_location after every successful hop.
        """
        global current_location, current_progress

        waypoints  = get_waypoints(start, target)
        total_legs = len(waypoints)

        if total_legs == 0:
            rospy.loginfo("[Nav] Already at target.")
            return True

        route_str = " → ".join(str(n) for n in [start] + waypoints)
        rospy.loginfo(f"[Nav] Route: {route_str}")

        current_progress = 0

        for idx, node in enumerate(waypoints):
            success = self.navigate_to_node(node, total_legs, idx)
            if not success:
                rospy.logwarn(f"[Nav] Route aborted at Node {node}.")
                return False
            current_location = node   # update location after each hop

        return True


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

    waypoints  = get_waypoints(start, target)
    route_info = " → ".join(str(n) for n in [start] + waypoints)

    def run_and_finish(s, t):
        global is_navigating, current_location, current_progress
        current_progress = 0
        is_navigating    = True
        success = my_robot.execute_path(s, t)
        is_navigating = False
        if success:
            current_location = t

    threading.Thread(target=run_and_finish, args=(start, target)).start()
    return jsonify({
        "status":      "starting",
        "target_node": target,
        "route":       route_info,
        "target_pose": NODE_POSES[target],
    }), 200


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


@app.route('/route/preview', methods=['POST'])
def preview_route():
    """Dry-run: return the planned waypoint list without moving."""
    data   = request.json or {}
    start  = data.get('start')
    target = data.get('target')
    if start not in NODE_POSES or target not in NODE_POSES:
        return jsonify({"status": "error", "message": "Invalid node id"}), 400
    waypoints  = get_waypoints(start, target)
    route      = [start] + waypoints
    return jsonify({"start": start, "target": target,
                    "route": route,
                    "route_str": " → ".join(str(n) for n in route)}), 200


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
    print("  POST /command              {start, target}")
    print("  GET  /status")
    print("  POST /stop")
    print("  POST /command/reset-home   → Reset AMCL to Node 1")
    print("  POST /amcl/set-pose        {x, y, yaw_deg}")
    print("  GET  /amcl/status")
    print("  GET  /nodes")
    print("  POST /nodes/<id>           {x, y, yaw_deg}")
    print("  POST /route/preview        {start, target}  ← dry-run routing")
    print()
    print("  Routing rules:")
    print("    1 → 2  :  1 → 0 → 2")
    print("    1 → 3  :  1 → 0 → 3")
    print("    2 → 1  :  2 → 0 → 1")
    print("    3 → 1  :  3 → 0 → 1")
    print("    2 → 3  :  2 → 3  (direct)")
    print("    3 → 2  :  3 → 2  (direct)")
    app.run(host='0.0.0.0', port=5000, debug=False, threaded=True)
