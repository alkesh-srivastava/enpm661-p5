#! /usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
# from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist
import math
import time

def euler_from_quaternion(x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)

        return roll_x, pitch_y, yaw_z # in radians

x1 = 0
y1 = 0
theta1 = 0
x2 = 0
y2 = 0
theta2 = 0



def newOdom1(msg):
    global x1
    global y1
    global theta1


    x1 = msg.pose.pose.position.x
    y1 = msg.pose.pose.position.y

    rot_q1 = msg.pose.pose.orientation
    _,_,theta1 = euler_from_quaternion(rot_q1.x, rot_q1.y, rot_q1.z, rot_q1.w)
    theta1 = theta1 * 180/3.14

def newOdom2(msg):
    global x2
    global y2
    global theta1

    x2 = msg.pose.pose.position.x
    y2 = msg.pose.pose.position.y

    rot_q2 = msg.pose.pose.orientation
    _,_,theta2 = euler_from_quaternion(rot_q2.x, rot_q2.y, rot_q2.z, rot_q2.w)
    theta2 = theta2 * 180/3.14

# one robot path here
robot1_path = [(99, 0), (98, 1), (97, 2), (96, 3), (95, 4), (94, 4), (93, 5), (92, 6), (91, 7), (90, 8), (89, 9), (88, 10), (87, 11), (86, 12), (85, 13), (84, 14), (83, 15), (83, 15), (82, 16), (81, 17), (80, 18), (79, 19), (78, 19), (77, 19), (76, 19), (75, 19), (74, 19), (73, 19), (72, 20), (71, 21), (70, 22), (69, 23), (68, 24), (67, 25), (66, 26), (65, 27), (64, 28), (63, 29), (62, 30), (61, 31), (60, 32), (59, 33), (58, 34), (57, 35), (56, 36), (55, 37), (54, 38), (53, 39), (52, 40), (51, 41), (50, 42), (49, 42), (48, 42), (47, 42), (46, 42), (45, 42), (44, 42), (43, 42), (42, 42), (41, 42), (40, 42), (39, 42), (38, 42), (37, 43), (36, 44), (35, 45), (34, 46), (33, 47), (32, 48), (32, 49), (32, 50), (32, 51), (32, 52), (32, 53), (32, 54), (31, 55), (31, 56), (31, 57), (31, 58), (31, 59), (31, 60), (31, 61), (31, 62), (31, 63), (30, 64), (29, 65), (29, 66), (29, 67), (29, 68), (29, 69), (29, 70)]
# two robot path here
robot2_path = [(99, 25), (98, 25), (98, 25), (98, 25), (98, 25), (98, 25), (98, 25), (98, 25), (97, 25), (96, 25), (95, 25), (94, 24), (93, 23), (92, 22), (91, 21), (90, 20), (90, 20), (90, 20), (90, 20), (90, 20), (89, 19), (88, 19), (87, 19), (86, 19), (85, 19), (84, 19), (83, 19), (83, 19), (82, 19), (81, 19), (80, 19), (79, 19), (78, 19), (77, 19), (76, 19), (75, 19), (74, 19), (73, 19), (72, 20), (71, 21), (70, 22), (69, 23), (68, 24), (67, 25), (66, 25), (65, 25), (64, 25), (63, 25), (62, 25), (61, 25), (60, 25), (59, 25), (58, 25), (57, 25), (56, 25), (55, 25), (54, 25), (53, 25), (52, 25), (51, 25), (50, 25), (49, 25), (48, 25), (47, 25), (46, 25), (45, 25), (44, 25), (43, 25), (42, 25), (41, 25), (40, 25), (39, 25), (38, 25), (37, 25), (36, 25), (35, 25), (34, 25), (34, 25), (34, 25), (34, 25), (34, 25), (34, 25), (34, 25), (33, 25), (32, 25), (31, 25), (30, 25), (29, 25)]
# Convert Path so that ROS topic can understand it

robot1_path_ros = []
for point in robot1_path:
    print("Generating ROS path")
    x = round(float(point[0]/10),1) - 9.9
    y = round(float(point[1]/10),1)
    robot1_path_ros.append((x,y))

robot2_path_ros = []
for point in robot2_path:
    print("Generating ROS path")
    x = round(float(point[0]/10),1) - 9.9
    y = round(float(point[1]/10),1)
    robot2_path_ros.append((x,y))

print(robot1_path_ros)


rospy.init_node("multi_robot_controller")

sub_1 = rospy.Subscriber("nexus1/odom", Odometry, newOdom1)

pub_1 = rospy.Publisher("nexus1/cmd_vel", Twist, queue_size = 1)

sub_2 = rospy.Subscriber("nexus2/odom", Odometry, newOdom2)

pub_2 = rospy.Publisher("nexus2/cmd_vel", Twist, queue_size = 1)

rate = rospy.Rate(10)

vel = Twist()
vel2 = Twist()

i = 0
r1_done = True
r2_done = True
while not rospy.is_shutdown():
    if math.sqrt(math.pow(robot1_path_ros[i+1][0] - x1, 2) +\
     math.pow(robot1_path_ros[i+1][1] - y1, 2)) < 0.03:
        print("Distance Travelled")
        i+=1
    if i+1 == len(robot1_path_ros):
        rospy.sleep()
        break

    print("Distance", math.sqrt(math.pow(robot1_path_ros[i+1][0] - x1, 2) +\
     math.pow(robot1_path_ros[i+1][1] - y1, 2)))

    vel.linear.x = robot1_path_ros[i+1][0] - robot1_path_ros[i][0]
    vel.linear.y = robot1_path_ros[i+1][1] - robot1_path_ros[i][1]
    pub_1.publish(vel)
    vel2.linear.x = robot2_path_ros[i+1][0] - robot2_path_ros[i][0]
    vel2.linear.y = robot2_path_ros[i+1][1] - robot2_path_ros[i][1]
    pub_2.publish(vel2)
    rate.sleep()








