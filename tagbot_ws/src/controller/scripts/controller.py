#!/usr/bin/env python
# coding:utf-8
import math
import numpy as np
import time
from common import *
from time import sleep
from std_msgs.msg import Bool
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3, Vector3Stamped
from sensor_msgs.msg import LaserScan
from dynamic_reconfigure.server import Server
from yahboomcar_laser.cfg import laserAvoidPIDConfig
RAD2DEG = 180 / math.pi

class laserAvoid:
    def __init__(self):
        rospy.on_shutdown(self.cancel)
        self.r = rospy.Rate(20)
        self.Moving = False
        self.switch = False
        self.Right_warning = 0
        self.Left_warning = 0
        self.front_warning = 0
        self.ros_ctrl = ROSCtrl()
	self.lin_pid = SinglePID(2.0, 0.0, 2.0)
        self.ang_pid = SinglePID(3.0, 0.0, 5.0)
        Server(laserAvoidPIDConfig, self.dynamic_reconfigure_callback)
        self.linear = 0.5
        self.angular = 1.0
        self.ResponseDist = 0.55 # the threshold distance 
        self.LaserAngle = 30  # 10~180
        self.sub_laser = rospy.Subscriber('/scan', LaserScan, self.registerScan, queue_size=1)
	
	self.tagbot_distance = 0
	self.tagbot_angle = 0
	self.timestamp = rospy.Time.now()
	self.sub_position = rospy.Subscriber('/tagbot/target_position', Vector3Stamped, self.register_position, queue_size = 1)

	self.sub_mytopic = rospy.Subscriber('/mytopic', Bool, self.callback, queue_size=1)
	self.detected = rospy.Subscriber('/tagbot/detected', Bool, self.detected_callback, queue_size=1)
	#self.tagbot_angle = rospy.Subscriber('/tagbot/angle', Float32, self.tagbot_angle_callback, queue_size=1)
	#self.tagbot_distance = rospy.Subscriber('/tagbot/distance', Float32, self.tagbot_distance_callback, queue_size=1)
	#self.sub_tagbot_timestamp = rospy.Subscriber('/tagbot/timestamp', Time, self.tagbot_timestamp_callback, queue_size=1)	
	self.active = False

    #def tagbot_timestamp_callback(self, data):
	#rospy.loginfo('my_message:{}'.format(data))
	#self.timestamp = data

    def register_position(self, message):
	if not isinstance(message, Vector3Stamped): return	
	timestamp = message.header.stamp
	self.tagbot_distance = message.vector.x / 1000
	self.tagbot_angle = message.vector.y
	print('received: distance: {},angle: {}'.format(self.tagbot_distance, self.tagbot_angle))
	

    def tagbot_angle_callback(self, data):
	rospy.loginfo('my_message:{}'.format(data))
	self.tagbot_angle = data

    def tagbot_distance_callback(self, data):
	rospy.loginfo('my_message:{}'.format(data))
	self.tagbot_distance = data	

    def detected_callback(self, data):
	rospy.loginfo('my_message:{}'.format(data))
	self.detected = data
    
    def callback(self, data):
	rospy.loginfo('my_message:{}'.format(data))
	self.active = data

    def cancel(self):
        self.ros_ctrl.pub_vel.publish(Twist())
        self.ros_ctrl.cancel()
        self.sub_laser.unregister()
        rospy.loginfo("Shutting down this node.")

    def dynamic_reconfigure_callback(self, config, level):
        self.switch = config['switch']
        self.linear = config['linear']
        self.angular = config['angular']
        self.LaserAngle = config['LaserAngle']
        self.ResponseDist = config['ResponseDist']
        return config

    def registerScan(self, scan_data):
	#print('my_message:{}'.format(self.active))
        #print('Moving:{}'.format(self.Moving))
        if not isinstance(scan_data, LaserScan): return
        # Record the laser scan and publish the position of the nearest object (or point to a point)
        ranges = np.array(scan_data.ranges)
        self.Right_warning = 0
        self.Left_warning = 0
        self.front_warning = 0
        # if we already have a last scan to compare to
        for i in range(len(ranges)):
            angle = (scan_data.angle_min + scan_data.angle_increment * i) * RAD2DEG
            # if angle > 90: print "i: {},angle: {},dist: {}".format(i, angle, scan_data.ranges[i])
            if 160 > angle > 180 - self.LaserAngle:
                if ranges[i] < self.ResponseDist: self.Right_warning += 1
            if - 160 < angle < self.LaserAngle - 180:
                if ranges[i] < self.ResponseDist: self.Left_warning += 1
            if abs(angle) > 160:
                if ranges[i] <= self.ResponseDist: self.front_warning += 1
        # print (self.Left_warning, self.front_warning, self.Right_warning)
        if self.ros_ctrl.Joy_active or self.switch == True or self.active == False:
            if self.Moving == True:
                self.ros_ctrl.pub_vel.publish(Twist())
                self.Moving = not self.Moving
            return
	#if self.Moving == True and self.active == False:
	#    self.Moving = False
	#    #self.ros_ctrl.pub_vel.publish(Twist())
	#    return
        self.Moving = True
        twist = Twist()
	print("check point1!")
	#temp
	if (True):#rospy.Time.now() - rospy.Duration(1)) < self.timestamp:
	    print("target detected!")
	    print('distance: {},angle: {}'.format(self.tagbot_distance, self.tagbot_angle))
	    if abs(self.tagbot_distance - self.ResponseDist) < 0.1: 
	        self.tagbot_distance = self.ResponseDist
                #if minDist - self.ResponseDist < 0.1: 
	        #self.tagbot_distance = self.ResponseDist
	    twist.linear.x = -self.lin_pid.pid_compute(self.ResponseDist, self.tagbot_distance)
	    ang_pid_compute = self.ang_pid.pid_compute((180 - abs(self.tagbot_angle)) / 72, 0)
	    if self.tagbot_angle > 0: twist.angular.z = -ang_pid_compute
	    else: twist.angular.z = ang_pid_compute
	    if ang_pid_compute < 0.02: twist.angular.z = 0
		
		
	    self.ros_ctrl.pub_vel.publish(twist)
	    
	    sleep(0.2)
        # Left positive and right negative
        if self.front_warning > 10 and self.Left_warning > 10 and self.Right_warning > 10:
            # print ('1, there are obstacles in the left and right, turn right')
            twist.linear.x = -0.15
            twist.angular.z = -self.angular
            self.ros_ctrl.pub_vel.publish(twist)
            sleep(0.2)
        elif self.front_warning > 10 and self.Left_warning <= 10 and self.Right_warning > 10:
            # print ('2, there is an obstacle in the middle right, turn left')
            twist.linear.x = 0
            twist.angular.z = self.angular
            self.ros_ctrl.pub_vel.publish(twist)
            sleep(0.2)
            if self.Left_warning > 10 and self.Right_warning <= 10:
                # print ('3, there is an obstacle on the left, turn right')
                twist.linear.x = 0
                twist.angular.z = -self.angular
                self.ros_ctrl.pub_vel.publish(twist)
                sleep(0.5)
        elif self.front_warning > 10 and self.Left_warning > 10 and self.Right_warning <= 10:
            # print ('4. There is an obstacle in the middle left, turn right')
            twist.linear.x = 0
            twist.angular.z = -self.angular
            self.ros_ctrl.pub_vel.publish(twist)
            sleep(0.2)
            if self.Left_warning <= 10 and self.Right_warning > 10:
                # print ('5, there is an obstacle on the left, turn left')
                twist.linear.x = 0
                twist.angular.z = self.angular
                self.ros_ctrl.pub_vel.publish(twist)
                sleep(0.5)
        elif self.front_warning > 10 and self.Left_warning < 10 and self.Right_warning < 10:
            # print ('6, there is an obstacle in the middle, turn left')
            twist.linear.x = 0
            twist.angular.z = self.angular
            self.ros_ctrl.pub_vel.publish(twist)
            sleep(0.2)
        elif self.front_warning < 10 and self.Left_warning > 10 and self.Right_warning > 10:
            # print ('7. There are obstacles on the left and right, turn right')
            twist.linear.x = 0
            twist.angular.z = -self.angular
            self.ros_ctrl.pub_vel.publish(twist)
            sleep(0.4)
        elif self.front_warning < 10 and self.Left_warning > 10 and self.Right_warning <= 10:
            # print ('8, there is an obstacle on the left, turn right')
            twist.linear.x = 0
            twist.angular.z = -self.angular
            self.ros_ctrl.pub_vel.publish(twist)
            sleep(0.2)
        elif self.front_warning < 10 and self.Left_warning <= 10 and self.Right_warning > 10:
            # print ('9, there is an obstacle on the right, turn left')
            twist.linear.x = 0
            twist.angular.z = self.angular
            self.ros_ctrl.pub_vel.publish(twist)
            sleep(0.2)

	#if object detected and no warnings, move to object
	elif (rospy.Time.now() - rospy.Duration(1)) < self.timestamp and self.front_warning <= 10 and self.Left_warning <= 10 and self.Right_warning <= 10:
	    print("target detected!")
	    print('distance: {},angle: {}'.format(self.tagbot_distance, self.tagbot_angle))
	    if abs(self.tagbot_distance - self.ResponseDist) < 0.1: 
                #if minDist - self.ResponseDist < 0.1: 
	        #self.tagbot_distance = self.ResponseDist

	        velocity.linear.x = -self.lin_pid.pid_compute(self.ResponseDist, self.tagbot_distance)
	        ang_pid_compute = self.ang_pid.pid_compute((180 - abs(tagbot_angle)) / 72, 0)
	    if tagbot_angle > 0: velocity.angular.z = -ang_pid_compute
	    else: velocity.angular.z = ang_pid_compute
	    if ang_pid_compute < 0.02: velocity.angular.z = 0
		
		
	    self.ros_ctrl.pub_vel.publish(velocity)
	    
	    sleep(0.2)

        elif self.front_warning <= 10 and self.Left_warning <= 10 and self.Right_warning <= 10:
            # print ('10, no obstacles, go forward')
            twist.linear.x = self.linear
            twist.angular.z = 0
            self.ros_ctrl.pub_vel.publish(twist)
	
	
        self.r.sleep()
        # else : self.ros_ctrl.pub_vel.publish(Twist())


if __name__ == '__main__':
    print('controller:hello')
    rospy.init_node('laser_Avoidance', anonymous=False)
    tracker = laserAvoid()
    rospy.spin()
