#!/usr/bin/env python

#This code is compiled in fulfillment of the COMS4030A - Robotics Course by:
#   Clerence ()
#   Gabriel Tidy (2152375)
#   Rotondwa Mavhengani (2114834)
#   and Samuel Oladejo (2441199)

'''
In this project we used SLAM through gmapping, motion planning with PRM and PID control
to ensure that a simulated turtlebot reached a user defined goal location while avoiding obstacles
in the environment.

***IMPORTANT***
Please go through the readme file before attempting to run this code. Incase of any errors, please reach out.  
We are available 24/7.
'''

import rospy
from geometry_msgs.msg import Twist, Point
import tf
from math import sqrt, pow, pi, atan2
from tf.transformations import euler_from_quaternion
import numpy as np
import math
from gazebo_msgs.srv import GetModelState
import networkx as nx
import matplotlib.pyplot as plt

kp_distance = 1
ki_distance = 0
kd_distance = 0

kp_angle = 1
ki_angle = 0
kd_angle = 0

class GotoPoint():
    def __init__(self, final_position):
        rospy.init_node('turtlebot3_pointop_key', anonymous=False)
        rospy.on_shutdown(self.shutdown)
        self.cmd_vel = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=10)
        position = Point()
        move_cmd = Twist()
        r = rospy.Rate(10)
        self.tf_listener = tf.TransformListener()
        self.odom_frame = 'odom'

        #returns the transform between two coordinate frames: odom_frame and base_frame = 'base_footprint' or 'base_link'
        try:
            self.tf_listener.waitForTransform(self.odom_frame, 'base_footprint', rospy.Time(), rospy.Duration(1.0))
            self.base_frame = 'base_footprint'
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            try:
                self.tf_listener.waitForTransform(self.odom_frame, 'base_link', rospy.Time(), rospy.Duration(1.0))
                self.base_frame = 'base_link'
            except (tf.Exception, tf.ConnectivityException, tf.LookupException):
                rospy.loginfo("Cannot find transform between odom and base_link or base_footprint")
                rospy.signal_shutdown("tf Exception")

        #gets the current position from the get_odom function
        (position, rotation) = self.get_odom()


        last_rotation = 0
        linear_speed = 3.5    #kp_distance
        angular_speed = 3.5  #kp_angular

        #stores the goal position
        goal_x = final_position[0]
        goal_y = final_position[1]
        goal_z = final_position[2]

        #this ensures that the angle z is always between +/- 180
        if goal_z > 180 or goal_z < -180:
            print("you input wrong z range.")
            self.shutdown()
        #converts the goal angle from degrees to radians to ease computation
        goal_z = np.deg2rad(goal_z)

        #compute the distance between the goal position and the current position
        goal_distance = sqrt(pow(goal_x - position.x, 2) + pow(goal_y - position.y, 2))

        #distance is the error for length, x,y
        distance = goal_distance
        previous_distance = 0
        total_distance = 0

        previous_angle = 0
        total_angle = 0
       
       #ensure that the distance is substansial enough for the robot to move
        while distance > 0.05:
            #get the current position
            (position, rotation) = self.get_odom()
            x_start = position.x
            y_start = position.y
            #path_angle = error
            path_angle = atan2(goal_y - y_start, goal_x- x_start)

            if path_angle < -pi/4 or path_angle > pi/4:
                if goal_y < 0 and y_start < goal_y:
                    path_angle = -2*pi + path_angle
                elif goal_y >= 0 and y_start > goal_y:
                    path_angle = 2*pi + path_angle
            if last_rotation > pi-0.1 and rotation <= 0:
                rotation = 2*pi + rotation
            elif last_rotation < -pi+0.1 and rotation > 0:
                rotation = -2*pi + rotation

            #Updates the distance and the angle
            diff_angle = path_angle - previous_angle
            diff_distance = distance - previous_distance

            distance = sqrt(pow((goal_x - x_start), 2) + pow((goal_y - y_start), 2))

            #computes the PID using the the distance at each time step
            control_signal_distance = kp_distance*distance + ki_distance*total_distance + kd_distance*diff_distance
            control_signal_angle = kp_angle*path_angle + ki_angle*total_angle + kd_distance*diff_angle

            #move the robot to the goal position with the updated PID control signal
            move_cmd.angular.z = (control_signal_angle) - rotation
            #move the robot to the goal orientation with the updated PID control signal
            move_cmd.linear.x = min(control_signal_distance, 0.1)


            if move_cmd.angular.z > 0:
                move_cmd.angular.z = min(move_cmd.angular.z, 1.5)
            else:
                move_cmd.angular.z = max(move_cmd.angular.z, -1.5)

            last_rotation = rotation
            #publish the rostopic
            self.cmd_vel.publish(move_cmd)
            r.sleep()
            previous_distance = distance
            total_distance = total_distance + distance

        (position, rotation) = self.get_odom()

        print("Reached position:\nx = " + str(position.x) + "\ny = " + str(position.y))

        #rospy.loginfo("Stopping the robot...")
        self.cmd_vel.publish(Twist())
        return

    #function to get the current position and rotation of the turtlebot
    def get_odom(self):
        try:
            #gets the robot position
            (trans, rot) = self.tf_listener.lookupTransform(self.odom_frame, self.base_frame, rospy.Time(0))
            #converts the robot orientation to the worlds orientation
            rotation = euler_from_quaternion(rot)

        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            rospy.loginfo("TF Exception")
            return
        return (Point(*trans), rotation[2])

    #shutdown the robot
    def shutdown(self):
        self.cmd_vel.publish(Twist())
        rospy.sleep(1)

def navigate_to_point(x_final, y_final):

    # generate input values
    coord = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
    resp_coord = coord('mobile_base','ground_plane')

    #gets the robots current orientation
    x = resp_coord.pose.orientation.x 
    y = resp_coord.pose.orientation.y
    z = resp_coord.pose.orientation.z
    w = resp_coord.pose.orientation.z

    # Convert a quaternion into euler angles (roll, pitch, yaw)
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)
        
    #Convert Radius to Degrees
    Z_in_deg = yaw_z * 180 / math.pi

    angle_final = Z_in_deg

    final = [x_final, y_final, angle_final]
    final_position = np.array(final)
    
    #Calling the Class to move the Turtlebot
    GotoPoint(final_position)



#grid map constants
EMPTY_SPACE = 255
OBSTACLE = 0
LOCAL_PATHS = 225
LANDMARK_POINTS = 10
FINAL_PATH = 60
PATH_WAYPOINTS = 10

#transformation matrix constants
X_SCALE_CONSTANT = 19.377
X_ADDITION_CONSTANT = 290
Y_SCALE_CONSTANT = -19.377
Y_ADDITION_CONSTANT = 234

#determines size of path collision checking
ROBOT_WIDTH = 7

#sampling iteration constants
INITIAL_NEAREST_NODES = 8
INITIAL_NUM_SAMPLES = 1000
MAX_NEAREST_NODES = 8
NUM_NEW_SAMPLES = 100
MAX_SAMPLES = 5000

#converts [x, y] world coordinates to map coordinates
def world_to_map_coordinates(world_coordinates):
    return np.array([int((world_coordinates[0] * X_SCALE_CONSTANT) + X_ADDITION_CONSTANT), int((world_coordinates[1] * Y_SCALE_CONSTANT) + Y_ADDITION_CONSTANT)])

#converts [x, y] map coordinates to world coordinates
def map_to_world_coordinates(map_coordinates):
    return np.array([(map_coordinates[0] - X_ADDITION_CONSTANT) / X_SCALE_CONSTANT, (map_coordinates[1] - Y_ADDITION_CONSTANT) / Y_SCALE_CONSTANT])

#opens map image file as numpy array, adds initial and goal milestones
def initialize_grid_map(initial_pos, goal_pos):
    with open("refined_map.pgm", 'rb') as map_pgm:
        grid_map = plt.imread(map_pgm)
    
    grid_map[initial_pos[1]][initial_pos[0]] = LANDMARK_POINTS
    grid_map[goal_pos[1]][goal_pos[0]] = LANDMARK_POINTS
    
    return grid_map, [initial_pos, goal_pos]

#checks whether a point on the grid map has a specific value (check_value) - typically used to check for obstacle intersections
def check_point(grid_map, x, y, check_value):
    map_size = grid_map.shape
    #ensure x and y within grid map bounds
    return (y >= 0 and y < map_size[0]) and (x >= 0 and x < map_size[1]) and grid_map[y][x] == check_value

#checks an area (determined by the robot width) around a point for obstacle collisions
def check_point_collisions(grid_map, x, y, check_value=OBSTACLE, robot_width=ROBOT_WIDTH):
    for i in range(-robot_width, robot_width+1):
        for j in range(-robot_width, robot_width+1):
            if (check_point(grid_map, x + j, y + i, check_value)):
                return True
    return False

#generates multiple random milestones, and adds them to the grid map provided it lies in free space
def sample_milestones(grid_map, milestones, num_samples):
    random_uniform_y_samples = np.random.randint(0, grid_map.shape[0], num_samples)
    random_uniform_x_samples = np.random.randint(0, grid_map.shape[1], num_samples)
    for point in range(num_samples):
        if (check_point_collisions(grid_map, random_uniform_x_samples[point], random_uniform_y_samples[point], check_value=EMPTY_SPACE)):
            milestones.append([random_uniform_x_samples[point], random_uniform_y_samples[point]])
            grid_map[random_uniform_y_samples[point]][random_uniform_x_samples[point]] = LANDMARK_POINTS

#used to find closest neighbouring milestones, and compare shortest path distances
def euclidean_distance_check(point1, point2):
    return np.sqrt(pow(point1[0] - point2[0], 2) + pow(point1[1] - point2[1], 2))
   
#checks for obstacle collisions along a path between two milestones   
def check_local_path_collisions(grid_map, input_node1, input_node2):
    if (input_node1[0] > input_node2[0]):#reverse order to make sure node1 is leftmost
        node1 = np.copy(input_node2)
        node2 = np.copy(input_node1)
    else:
        node1 = np.copy(input_node1)
        node2 = np.copy(input_node2)

    x,y = node1
    distance = 0
    while (x < node2[0]):
        if (y < node2[1]):
            y += 1
        elif (y > node2[1]):
            y -= 1
        x += 1
        distance += 1
        if check_point_collisions(grid_map, x, y):
            return False, distance
    while (y != node2[1]):
        if (y < node2[1]):
            y += 1
        else:
            y -= 1
        distance += 1
        if check_point_collisions(grid_map, x, y):
            return False, distance
        
    return True, distance

#builds a networkx graph of each valid milestone and their connected edges
def build_graph(grid_map, milestones, num_nearest_nodes):
    node_graph = nx.Graph()
    num_nodes = len(milestones)
    nearest_node_indeces = np.ones((num_nodes, num_nearest_nodes), dtype=int) * num_nodes#multiply by num nodes so that unitialized indeces raise out of bounds error
    for m1 in range(num_nodes):
        node_graph.add_node(m1)
        node_distances = np.ones(num_nodes) * float("inf")
        for m2 in range(num_nodes):
            if (m1 != m2):
                node_distances[m2] = euclidean_distance_check(milestones[m1], milestones[m2])
        nearest_node_indeces[m1] = np.argpartition(node_distances, num_nearest_nodes)[:num_nearest_nodes]
    
    milestones = np.array(milestones)
    for m1 in range(num_nodes):
        n1 = milestones[m1]
        for m2 in range(num_nearest_nodes):
            n2 = milestones[nearest_node_indeces[m1]][m2]
            valid_local_path, local_path_distance = check_local_path_collisions(grid_map, n1, n2)
            if valid_local_path:
                node_graph.add_edge(m1, nearest_node_indeces[m1][m2], weight=local_path_distance)
    
    return node_graph

#uses the above functions to find a valid path between the initial and goal milestones
def get_shortest_path(OG_grid_map, OG_milestones, num_nearest_nodes, num_samples):
    grid_map = np.copy(OG_grid_map)
    milestones = list(np.copy(OG_milestones))
    sample_milestones(grid_map, milestones, num_samples)
    node_graph = build_graph(grid_map, milestones, num_nearest_nodes)

    if (1 in nx.algorithms.descendants(node_graph, 0)):#if valid path exists
        shortest_path = nx.shortest_path(node_graph, 0, 1, weight="weight")#find shortest valid path
        return shortest_path, grid_map, milestones, node_graph, num_nearest_nodes, num_samples
    
    if (num_nearest_nodes < MAX_NEAREST_NODES):#if no valid path found, resample milestones
        num_nearest_nodes += 1
    if (num_samples < MAX_SAMPLES):
        num_samples += NUM_NEW_SAMPLES
    else:
        return [], grid_map, milestones, node_graph, -1, -1

    # print(num_nearest_nodes, num_samples)

    return get_shortest_path(OG_grid_map, OG_milestones, num_nearest_nodes, num_samples)

#uses the above functions to get a list of waypoints (in world cooridinates) along a valid path from the initial to the goal position
def get_waypoints_for_PID(world_initial_position, world_goal_position):
    #convert input coordinates to map coordinates
    map_initial_coordinates = world_to_map_coordinates(world_initial_position)
    map_goal_coordinates = world_to_map_coordinates(world_goal_position)

    grid_map, milestones = initialize_grid_map(map_initial_coordinates, map_goal_coordinates)
    shortest_path, grid_map, milestones, node_graph, num_nearest_nodes, num_samples = get_shortest_path(grid_map, milestones, INITIAL_NEAREST_NODES, INITIAL_NUM_SAMPLES)

    PID_waypoints = []
    if (num_nearest_nodes != -1 and num_samples != -1):
        for node in range(len(shortest_path) - 1):
            PID_waypoints.append(map_to_world_coordinates(milestones[shortest_path[node]]))#convert milestone coordinates to world coordinates
        PID_waypoints.append(map_to_world_coordinates(milestones[shortest_path[-1]]))
        print("Path (" + str(len(PID_waypoints)) + " waypoints) found using " + str(num_samples) + " random samples with " + str(num_nearest_nodes) + " nearest milestones each.")
        return True, PID_waypoints

    return False, PID_waypoints



while True:
    #gets initial position from gazebo state service
    coord = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
    resp_coord = coord('mobile_base','ground_plane')
    x_pose = resp_coord.pose.position.x 
    y_pose = resp_coord.pose.position.y
    initial_position = [x_pose, y_pose]

    #gets goal position as input from terminal
    goal_x = float(input("Enter Goal Destination - x: "))
    goal_y = float(input("Enter Goal Destination - y: "))
    goal_position = [goal_x, goal_y]

    #determines valid path using PRM
    print("Finding motion path using PRM (this may take a while)...")
    path_success, PID_waypoints = get_waypoints_for_PID(initial_position, goal_position)

    if path_success:
        print("\nStarting at:\nx = " + str(initial_position[0]) + "\ny = " + str(initial_position[1]) + "\nGoing to at:\nx = " + str(goal_position[0]) + "\ny = " + str(goal_position[1]))

        #navigates to each waypoint along path using PID control
        for w in range(len(PID_waypoints)):
            print("\nNavigating to waypoint " + str(w+1) + ":\nx = " + str(PID_waypoints[w][0]) + "\ny = " + str(PID_waypoints[w][1]))
            navigate_to_point(PID_waypoints[w][0], PID_waypoints[w][1])
        print("\nFinal destination reached.")

        #allows the program to be repeated with a new goal
        new_run = raw_input("\nContinue with another goal? (y/n): ")
        if new_run == "n" or new_run == "N" or new_run == "no" or new_run == "No" or new_run == "NO":
            print("Shutting down program...")
            break

    #if no valid path found, either valid path does not exist or not enough samples were used, allows the user to repeat their attempt
    else:
        new_run = raw_input("\nNo valid path found. Make sure goal destination is valid. Try again? (y/n): ")
        if new_run == "n" or new_run == "N" or new_run == "no" or new_run == "No" or new_run == "NO":
            print("Shutting down program...")
            break
