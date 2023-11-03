#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry, OccupancyGrid
from tf.transformations import euler_from_quaternion
import math
from Astar import AStar
import random
import time
from datetime import datetime, timedelta

class TurtleBot3Exploration:
    def __init__(self):
        rospy.init_node('turtlebot3_exploration_node', anonymous=True)

        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.map_sub = rospy.Subscriber('/map', OccupancyGrid, self.map_callback)

        self.explore_cmd = Twist()
        self.laser_data = None
        self.pose = None
        self.map_data = None
        self.map_resolution = None
        self.map_width = None
        self.map_height = None
        self.map_origin_x = None
        self.map_origin_y = None

    def scan_callback(self, scan_msg):
        self.laser_data = scan_msg.ranges

    def odom_callback(self, odom_msg):
        self.pose = odom_msg.pose.pose

    def map_callback(self, map_msg):
        self.map_data = map_msg.data
        self.map_resolution = map_msg.info.resolution
        self.map_width = map_msg.info.width
        self.map_height = map_msg.info.height
        self.map_origin_x = map_msg.info.origin.position.x
        self.map_origin_y = map_msg.info.origin.position.y

    def get_yaw_angle(self):
        orientation = self.pose.orientation
        orientation_list = [orientation.x, orientation.y, orientation.z, orientation.w]
        _, _, yaw = euler_from_quaternion(orientation_list)
        return yaw

    def convert_map_to_array(self):
    #Let's check if the map data is available
        if self.map_data is None or self.map_resolution is None or self.map_width is None or self.map_height is None:
            rospy.logwarn("Map data not available yet.")
            return False
        # a 2d array is initialized to store the map grid
        map_array = [[0 for _ in range(self.map_width)] for _ in range(self.map_height)]

        # Calculate the origin of the map in grid coordinates
        self.map_origin_x = self.map_width * self.map_resolution / 2
        self.map_origin_y = self.map_height * self.map_resolution / 2

        # let's cast the map into a 2d array
        for y in range(self.map_height):
            for x in range(self.map_width):
                # Calculate index in map data from grid coordinates
                map_index = y * self.map_width + x
                # Convert map data to 2D array format (obstacle = 1, free space = 0)
                if self.map_data[map_index] == 100:  # Occupied cell (obstacle)
                    map_array[y][x] = 1
                else:  # Free cell
                    map_array[y][x] = 0

        return map_array

    def a_star_algorithm(self, start_pos_x,start_pos_y, target_pos_x,target_pos_y, map_array):
        # let's call Astar algorithm with the 2d array of the map
        a_star = AStar(map_array)

        # Define start and goal positions in grid coordinates (e.g., from the odometry and target position)
        start = (start_pos_x, start_pos_y)
        goal = (target_pos_x, target_pos_y)

        # Find the path using A* algorithm
        path = a_star.a_star_search(start, goal)
        return path
    def is_obstacle_in_front(self):
        if self.laser_data:
            front_angles = range(-15, 16)
            front_distances = [self.laser_data[i] for i in front_angles]
            min_distance_front = min(front_distances)
            if min_distance_front <= 0.05:  # Threshold distance for obstacle in front
                return True
        return False
    def avoid(self, change):
        print("Found an obstacle, turtlebot is avoiding it...\n")
        rate = rospy.Rate(10)
        backward_duration = 1
        rotation_duration = 6
        forward_duration = 2
        #make the robot go backwards
        self.explore_cmd.linear.x = -0.2
        self.explore_cmd.angular.z = 0.0
        current_time = int(time.time())
        start_time = int(time.time())
        while (current_time - start_time) < backward_duration:
          self.cmd_vel_pub.publish(self.explore_cmd)
          rate.sleep()
          current_time = int(time.time())
        #make the robot rotate to avoid obstacle
        self.explore_cmd.linear.x = 0.0
        self.explore_cmd.angular.z = change/6
        current_time = int(time.time())
        start_time = int(time.time())
        while (current_time - start_time) < rotation_duration:
          self.cmd_vel_pub.publish(self.explore_cmd)
          rate.sleep()
          current_time = int(time.time())
        self.explore_cmd.linear.x = 0.2
        self.explore_cmd.angular.z = 0.0
        current_time = int(time.time())
        start_time = int(time.time())
        while (current_time - start_time) < forward_duration:
          self.cmd_vel_pub.publish(self.explore_cmd)
          rate.sleep()
          current_time = int(time.time())
        self.explore_cmd.linear.x = 0.0
        self.explore_cmd.angular.z = 0.0        
        self.cmd_vel_pub.publish(self.explore_cmd)
        rate.sleep()


    def navigate_path(self, path):
        rate = rospy.Rate(10)
        initial_time = datetime.now()

        while not rospy.is_shutdown() and path:
            current_pos_x, current_pos_y = path[0]
            # Calculate the difference between the current position and the target position
            dx = current_pos_x - self.pose.position.x
            dy = current_pos_y - self.pose.position.y
            distance_to_target = math.sqrt(dx*dx + dy*dy)

            if distance_to_target < 0.1:  # If the robot is close to the target waypoint, move to the next waypoint
                path.pop(0)  # Remove the current waypoint from the path
                continue

            # Calculate the angle between the current orientation and the target waypoint
            yaw = self.get_yaw_angle()
            target_angle = math.atan2(dy, dx)
            angle_difference = target_angle - yaw

            #robot's orientation is adjusted towards the target waypoint
            if abs(angle_difference) > 0.1:
                self.explore_cmd.linear.x = 0.0
                self.explore_cmd.angular.z = 0.2 * angle_difference
            else:
                # Move the robot towards the target waypoint
                self.explore_cmd.linear.x = 0.2
                self.explore_cmd.angular.z = 0.0
            if self.is_obstacle_in_front():
                self.avoid(1)
                break
            self.cmd_vel_pub.publish(self.explore_cmd)
            rate.sleep()
            #stops the navigation if it cant reach the goal after 5 secs, in case the goal is unreachable
            if datetime.now() - initial_time >= timedelta(seconds=5):
                break
        # Stop the robot once it reaches the final goal
        self.explore_cmd.linear.x = 0.0
        self.explore_cmd.angular.z = 0.0
        self.cmd_vel_pub.publish(self.explore_cmd)

    def explore(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            change = 1
            if self.laser_data and self.pose and self.map_data:
                # Get current robot position and convert it to grid coordinates
                current_pos_x = int((self.pose.position.x - self.map_origin_x) / self.map_resolution)
                current_pos_y = int((self.pose.position.y - self.map_origin_y) / self.map_resolution)
                #convert map into an array for using A*
                map_array = self.convert_map_to_array()

                #check if map exists and is already converted into an array for using A*
                if not map_array:
                    print("Sorry, no map available and/or there is data from SLAM\n")
                else:
                    if change == 1:
                        change = -1
                    else:
                        change = 1
                    if self.is_obstacle_in_front():
                        self.avoid(change)
                    current_time = int(time.time())
                    random.seed(current_time)
                    # Define a range for random exploration distance (e.g., within the boundaries of the map)
                    max_exploration_distance = min(self.map_width, self.map_height) / 2
                    # Generate random angle and distance for exploration
                    random_angle = random.uniform(0, 2 * math.pi)
                    random_distance = random.uniform(1.0, max_exploration_distance)

                    # Calculate the target position based on random angle and distance
                    target_pos_x = current_pos_x + int(random_distance * math.cos(random_angle))
                    target_pos_y = current_pos_y + int(random_distance * math.sin(random_angle))

                # Use the computed A* path for navigation
                path = self.a_star_algorithm(current_pos_x, current_pos_y, target_pos_x, target_pos_y, map_array)
                #check if there's a path
                if path:
                    print("turtlebot is exploring....\n")
                    self.navigate_path(path)
                else:
                    print("There's a map but there's not an available path\n")
            rate.sleep()

if __name__ == '__main__':
    try:
        turtlebot3_explore = TurtleBot3Exploration()
        turtlebot3_explore.explore()
    except rospy.ROSInterruptException:
        pass
