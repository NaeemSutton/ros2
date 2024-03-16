#!/usr/bin/env python3
import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped
import tf_transformations

# Hilbert curve generation functions
def rot(n, rx, ry, point):
    if not ry:
        if rx:
            point[0] = (n - 1) - point[0]
            point[1] = (n - 1) - point[1]
        point[0], point[1] = point[1], point[0]

def calcD(n, point):
    d = 0
    s = n >> 1
    while s > 0:
        rx = ((point[0] & s) != 0)
        ry = ((point[1] & s) != 0)
        d += s * s * ((3 * rx) ^ ry)
        rot(s, rx, ry, point)
        s >>= 1
    return d

def fromD(n, d):
    p = [0, 0]
    t = d
    s = 1
    while s < n:
        rx = ((t & 2) != 0)
        ry = (((t + (rx << 1)) & 1) != 0)
        rot(s, rx, ry, p)
        p[0] += s * rx
        p[1] += s * ry
        t >>= 2
        s <<= 1
    return p

def getPointsForCurve(order):
    points = []
    n = 1 << order
    for d in range(n * n):
        points.append(fromD(n, d))
    return points

def create_pose_stamped(position_x, position_y, rotation_z):
    q_x, q_y, q_z, q_w = tf_transformations.quaternion_from_euler(0.0, 0.0, rotation_z)
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.header.stamp = rclpy.time.Time().to_msg()  # Use rclpy time
    goal_pose.pose.position.x = position_x
    goal_pose.pose.position.y = position_y
    goal_pose.pose.position.z = 0.0
    goal_pose.pose.orientation.x = q_x
    goal_pose.pose.orientation.y = q_y
    goal_pose.pose.orientation.z = q_z
    goal_pose.pose.orientation.w = q_w
    return goal_pose

def main():
    # --- Init ROS2 communications and Simple Commander API ---
    rclpy.init()
    nav = BasicNavigator()

    # --- Set initial pose ---
    # !!! Comment if the initial pose is already set !!!
    initial_pose = create_pose_stamped(0.0, 0.0, 0.0)
    # nav.setInitialPose(initial_pose)

    # --- Wait for Nav2 ---
    nav.waitUntilNav2Active()

    # --- Generate Hilbert curve waypoints ---
    order = 3  # Example order for Hilbert curve
    hilbert_points = getPointsForCurve(order)

    waypoints = []
    for point in hilbert_points:
        pose = create_pose_stamped(point[0], point[1], 0.0)  # Assume rotation is 0 for simplicity
        waypoints.append(pose)

    # --- Follow Waypoints ---
    for waypoint in waypoints:
        nav.goToPose(waypoint)
        while not nav.isTaskComplete():
            feedback = nav.getFeedback()

    # --- Get the result ---
    print(nav.getResult())

    # --- Shutdown ROS2 communications ---
    rclpy.shutdown()

if __name__ == '__main__':
    main()
