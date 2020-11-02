#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool, Int8
import yaml
import numpy as np
from tf import transformations as tr
import tf
from geometry_msgs.msg import Twist
import math
import rospkg
from std_srvs.srv import Empty

class nav_path(object):
    """docstring for nav_path."""
    def __init__(self):
        super(nav_path, self).__init__()

        rospy.init_node('nav_pre_path_node')

        # Initial Pose
        self.pre_defined_path = []
        self.read_path()

        # Parameter
        self.static_v = 0.15
        self.static_omega = 0.1

        # Variables
        self.robot_pose = {'x': 0.0, 'y': 0.0, 'theta': 0.0}

        # Start Subscriber
        # sub_pose = rospy.Subscriber('robot_pose', PoseStamped, self.get_robot_pose, queue_size=10)
        # Start Publisher
        # self.pub_vel = rospy.Publisher('robot_vel', Twist, queue_size=5)
        self.pub_vel = rospy.Publisher('/X1/cmd_vel_relay', Twist, queue_size=5)

        # tf
        self.listener = tf.TransformListener()

        # Start Service Server
        srv_dis = rospy.Service('to_dispose', Empty, self.goto_dispose)
        srv_dis = rospy.Service('to_inspect', Empty, self.goto_inspect)

    def goto_dispose(self, req):

        segment = 1
        while not rospy.is_shutdown():
            try:
                (trans,rot) = self.listener.lookupTransform('X1/odom', 'X1/base_link', rospy.Time(0))
                self.robot_pose['x'] = trans[0]
                self.robot_pose['y'] = trans[1]
                roll, pitch, yaw = tr.euler_from_quaternion(rot)
                self.robot_pose['theta'] = yaw
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
                vel = self.compose_vel(0, 0)
                self.pub_vel(vel)
                raise
                continue

            if segment == 1:
                print("x: ",self.robot_pose['x'])
                print("path x: ",self.pre_defined_path[0][0])
                if self.robot_pose['x'] <= self.pre_defined_path[0][0]:
                    segment += 1
                    vel = self.compose_vel(0, 0)
                else:
                    vel = self.compose_vel(-self.static_v, 0)

            elif segment == 2:
                if self.robot_pose['theta'] >= math.pi/2 and self.robot_pose['theta'] <= math.pi/2+0.05:
                    segment += 1
                    vel = self.compose_vel(0, 0)
                else:
                    vel = self.compose_vel(0, self.static_omega)

            elif segment == 3:
                if self.robot_pose['y'] >= self.pre_defined_path[1][1]:
                    segment += 1
                    vel = self.compose_vel(0, 0)
                else:
                    vel = self.compose_vel(self.static_omega, 0)

            elif segment == 4:
                if self.robot_pose['theta'] <= 0 and self.robot_pose['theta'] >= 0-0.05:
                    segment += 1
                    vel = self.compose_vel(0, 0)
                else:
                    vel = self.compose_vel(0, -self.static_omega)

            elif segment == 5:
                if self.robot_pose['x'] >= self.pre_defined_path[2][0]:
                    rospy.loginfo("Robot Arrive")
                    vel = self.compose_vel(0, 0)
                    self.pub_vel.publish(vel)
                    break
                vel = self.compose_vel(self.static_v, 0)

            self.pub_vel.publish(vel)
            rospy.sleep(0.01)


    def goto_inspect(self, req):

    # def get_robot_pose(self, msg):
    #
    #     self.robot_pose.x = msg.pose.pose.position.x
    #     self.robot_pose.y = msg.pose.pose.position.y
    #     q = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
    #     roll, pitch, yaw = tr.euler_from_quaternion(q)
    #     self.robot_pose.theta = yaw
        pass
        
    def compose_vel(self, x, omega):
        v = Twist()
        v.linear.x = x
        v.linear.y = 0
        v.linear.z = 0
        v.angular.x = 0
        v.angular.y = 0
        v.angular.z = omega
        return v

    def read_path(self):
        self.path_file = rospy.get_param("nav_path", 'demo')
        rospack = rospkg.RosPack()
        self.path_file = rospack.get_path('move_base_example')+'/config/farm_demo.yaml'

        paths = yaml.load(file(self.path_file,'r'))
        self.pre_defined_path = np.array(paths['to_dispose'])
        self.pre_defined_path = np.vstack((self.pre_defined_path, paths['to_inspect']))

        print("My Path: ")
        print(self.pre_defined_path)

if __name__ == '__main__':
    d = nav_path()
    rospy.spin()
