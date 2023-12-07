#!/usr/bin/env python
# coding:utf-8
import math
import numpy as np
from common import *
from time import sleep
from geometry_msgs.msg import Vector3Stamped, Twist
from sensor_msgs.msg import LaserScan
from dynamic_reconfigure.server import Server
from yahboomcar_laser.cfg import laserAvoidPIDConfig
from Rosmaster_Lib import Rosmaster
RAD2DEG = 180 / math.pi
TAG_THRESHOLD = 0.6 # meters


class TagBotController:
    def __init__(self):
        rospy.on_shutdown(self.cancel)
        self.bot = Rosmaster()
        self.r = rospy.Rate(20)
        self.Moving = False
        self.switch = False
        self.right_warning = False
        self.left_warning = False
        self.front_warning = False
        self.ros_ctrl = ROSCtrl()
        self.lin_pid = SinglePID(2.0, 0.0, 2.0)
        self.ang_pid = SinglePID(3.0, 0.0, 5.0)
        Server(laserAvoidPIDConfig, self.dynamic_reconfigure_callback)
        self.linear = 0.5
        self.angular = 1.0
        self.prev_linear = 0
        self.latest_position = None
        self.sub_laser = rospy.Subscriber('/scan', LaserScan, self.register_scan, queue_size=1)
        self.sub_position = rospy.Subscriber('/tagbot/target_position', Vector3Stamped, self.register_position, queue_size = 1)

    def register_position(self, message):
        if not isinstance(message, Vector3Stamped): return
        if self.latest_position is not None and self.latest_position.header.stamp > message.header.stamp:
            return # Ignores old messages
        if message.vector.x == 0:
            return # Ignores invalid readings
        self.latest_position = message

    def is_target_detected(self):
        if self.latest_position is None: return False
        two_seconds_ago = rospy.Time.now() - rospy.Duration(2)
        return self.latest_position.header.stamp > two_seconds_ago

    def cancel(self):
        self.ros_ctrl.pub_vel.publish(Twist())
        self.ros_ctrl.cancel()
        self.sub_laser.unregister()
        self.sub_position.unregister()

    def dynamic_reconfigure_callback(self, config, level):
        self.switch = config['switch']
        self.linear = config['linear']
        self.angular = config['angular']
        self.laser_angle = config['LaserAngle']
        return config

    def register_scan(self, scan_data):
        if not isinstance(scan_data, LaserScan): return
        self.update_obstacle_warnings(scan_data)
        if self.is_obstacles_detected():
            # Turn away from the obstacles
            self.avoid_obstacles()
        else:
            if self.is_target_detected():
                # TODO If tagged
                # Chase the target
                self.chase_target()
            else:
                # Search the area
                pass
        # if self.ros_ctrl.Joy_active or self.switch == True:# or self.active == False:
        #     if self.Moving == True:
        #         self.ros_ctrl.pub_vel.publish(Twist())
        #         self.Moving = not self.Moving
        #         return
        self.r.sleep()

    def update_obstacle_warnings(self, scan_data):
        ranges = np.array(scan_data.ranges)
        right_count = left_count = front_count = 0
        for i in range(len(ranges)):
            angle = (scan_data.angle_min + scan_data.angle_increment * i) * RAD2DEG
            if 160 > angle > 70:
                if ranges[i] < TAG_THRESHOLD: # TODO Maybe set another distance threshold for obstacle avoidance
                    right_count += 1
            if -160 < angle < -70:
                if ranges[i] < TAG_THRESHOLD:
                    left_count += 1
            if abs(angle) > 160:
                if ranges[i] <= TAG_THRESHOLD:
                    front_count += 1
        self.right_warning = right_count > 10
        self.left_warning = left_count > 10
        self.front_warning = front_count > 10

    def is_obstacles_detected(self):
        return self.right_warning or self.left_warning or self.front_warning
    
    def move_robot(self, distance, angle):
        twist = Twist()
        twist.linear.x = self.prev_linear = distance
        twist.angular.z = angle
        self.ros_ctrl.pub_vel.publish(twist)
        sleep(0.2)

    def avoid_obstacles(self):
        if self.front_warning and self.left_warning and self.right_warning:
            # Back up and turn right
            self.move_robot(distance=-0.15, angle=-self.angular)
        elif self.front_warning and not self.left_warning and self.right_warning:
            # Turn left
            self.move_robot(distance=0, angle=self.angular)
        elif self.front_warning and self.left_warning and not self.right_warning:
            # Turn right
            self.move_robot(distance=0, angle=-self.angular)
        elif self.front_warning and not self.left_warning and not self.right_warning:
            # Turn left
            self.move_robot(distance=0, angle=self.angular)
        elif not self.front_warning and self.left_warning and self.right_warning:
            # Turn right
            self.move_robot(distance=0, angle=-self.angular)
            sleep(0.2)
        elif not self.front_warning and self.left_warning and not self.right_warning:
            # Turn right
            self.move_robot(distance=0, angle=-self.angular)
        elif not self.front_warning and not self.left_warning and self.right_warning:
            # Turn left
            self.move_robot(distance=0, angle=self.angular)

    def chase_target(self):
        target_distance = self.latest_position.vector.x / 1000.0
        target_angle = self.latest_position.vector.y
        distance = -self.lin_pid.pid_compute(TAG_THRESHOLD, target_distance)
        angle = (self.ang_pid.pid_compute((180 - abs(target_angle)) / 72.0, 0)) * 0.15
        if abs(angle) < 0.02:
            angle = 0
        self.move_robot(distance=distance, angle=angle)


if __name__ == '__main__':
    rospy.init_node('TagBotController', anonymous=False)
    tracker = TagBotController()
    rospy.spin()
