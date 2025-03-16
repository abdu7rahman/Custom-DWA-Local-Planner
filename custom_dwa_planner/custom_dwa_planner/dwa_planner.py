import rclpy
import math
import numpy as np
import random
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
from tf_transformations import euler_from_quaternion

goal_x = None
goal_y = None
goal_angle = None
odom_data = None
scan_data = None
goal_reached = False


def ask_for_goal():
    global goal_x, goal_y, goal_reached

    goal_x = float(input("Enter goal X : "))
    goal_y = float(input("Enter goal Y : "))
    goal_reached = False


def odom_callback(msg):
    global odom_data
    odom_data = msg


def scan_callback(msg):
    global scan_data
    scan_data = msg


def predict_motion(speed, turn_rate, step_time):
    if odom_data is None:
        return []

    x = odom_data.pose.pose.position.x
    y = odom_data.pose.pose.position.y
    orient = odom_data.pose.pose.orientation
    roll, pitch, yaw = euler_from_quaternion([orient.x, orient.y, orient.z, orient.w])

    path = []
    for i in range(100):
        yaw += turn_rate * step_time
        x += speed * math.cos(yaw) * step_time
        y += speed * math.sin(yaw) * step_time
        path.append((x, y))

    return path


def check_for_collisions(path):
    if scan_data is None:
        return -float('inf')

    safety_margin = 0.3

    for x, y in path:
        distance = math.sqrt(x**2 + y**2)
        scan_index = int(math.atan2(y, x) * (len(scan_data.ranges) / (2 * math.pi)))
        scan_index = max(0, min(len(scan_data.ranges) - 1, scan_index))

        if distance < scan_data.ranges[scan_index] - safety_margin:
            return -100000

    return 0


def choose_best_path(node, possible_paths):
    global goal_x, goal_y, goal_reached

    if odom_data is None:
        return 0.0, 0.0

    current_x = odom_data.pose.pose.position.x
    current_y = odom_data.pose.pose.position.y
    orient = odom_data.pose.pose.orientation
    roll, pitch, yaw = euler_from_quaternion([orient.x, orient.y, orient.z, orient.w])

    distance_to_goal = math.hypot(goal_x - current_x, goal_y - current_y)
    if distance_to_goal < 0.05:
        if not goal_reached:
            goal_reached = True
            node.get_logger().info(f"Goal reached at ({goal_x}, {goal_y})!")
        return 0.0, 0.0

    best_score = float('-inf')
    best_speed, best_turn = 0.05, 0.0

    for speed, turn, path in possible_paths:
        goal_distance_score = -math.hypot(path[-1][0] - goal_x, path[-1][1] - goal_y) * 5
        angle_diff = abs(math.atan2(goal_y - current_y, goal_x - current_x) - yaw)
        heading_score = -angle_diff * 2
        collision_risk = check_for_collisions(path)
        smoothness_score = -0.1 * abs(turn)

        total_score = goal_distance_score + heading_score + collision_risk + smoothness_score
        if total_score > best_score:
            best_score = total_score
            best_speed, best_turn = speed, turn

    return best_speed, best_turn



def generate_infinite_paths(max_speed, max_turn, step_time):
    while True:
        speed = random.uniform(0, max_speed)
        turn = random.uniform(-max_turn, max_turn)


        path = predict_motion(speed, turn, step_time)

        yield (speed, turn, path)


def movement_loop(node, cmd_publisher, path_publisher, max_speed, max_turn, step_time):
    global goal_reached

    if odom_data is None or scan_data is None or goal_reached:
        return


    path_generator = generate_infinite_paths(max_speed, max_turn, step_time)
    possible_paths = [next(path_generator) for i in range(10000)]

    speed, turn = choose_best_path(node, possible_paths)

    move_cmd = Twist()
    move_cmd.linear.x = speed
    move_cmd.angular.z = turn
    cmd_publisher.publish(move_cmd)

    marker = Marker()
    marker.header.frame_id = "base_link"
    marker.type = Marker.LINE_STRIP
    marker.action = Marker.ADD
    marker.scale.x = 0.01
    marker.color.r = 1.0
    marker.color.a = 0.5

    for _, _, path in possible_paths:
        for x, y in path:
            point = Point()
            point.x = x
            point.y = y
            marker.points.append(point)

    path_publisher.publish(marker)


def main():
    rclpy.init()
    node = Node('dwa_planner')

    ask_for_goal()

    node.create_subscription(Odometry, '/odom', odom_callback, 10)
    node.create_subscription(LaserScan, '/scan', scan_callback, 10)
    cmd_publisher = node.create_publisher(Twist, '/cmd_vel', 10)
    path_publisher = node.create_publisher(Marker, '/visual_paths', 10)

    max_speed = 0.15
    max_turn = 2.5
    step_time = 0.1

    node.create_timer(step_time, lambda: movement_loop(node, cmd_publisher, path_publisher, max_speed, max_turn, step_time))

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
