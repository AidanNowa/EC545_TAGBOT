#!/usr/bin/env python
# coding:utf-8
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Vector3, Vector3Stamped, Twist
from dynamic_reconfigure.server import Server
from yahboomcar_laser.cfg import laserTrackerPIDConfig

from common import ROSCtrl, SinglePID


class PIDController:
    def __init__(self):
        rospy.on_shutdown(self.on_shutdown)
        self.latest_position = None
        self.moving = False
        self.switch = False
        self.ros_ctrl = ROSCtrl()
        self.lin_pid = SinglePID(2.0, 0.0, 2.0)
        self.ang_pid = SinglePID(3.0, 0.0, 5.0)
        self.response_dist = rospy.get_param('~targetDist', 1.0)
        Server(laserTrackerPIDConfig, self.dynamic_reconfigure_callback)
        self.laser_angle = 90
        self.priority_angle = 30
        self.sub_laser = rospy.Subscriber('/scan', LaserScan, self.register_scan, queue_size=1)
        self.sub_position = rospy.Subscriber('/tagbot/target_position', Vector3Stamped, self.register_position, queue_size=1)

    def on_shutdown(self):
        self.ros_ctrl.pub_vel.publish(Twist())
        self.ros_ctrl.cancel()
        self.sub_laser.unregister()
        self.sub_position.unregister()

    def register_position(self, message):
        if not isinstance(message, Vector3Stamped): return
        if self.latest_position is not None and self.latest_position.header.stamp > message.header.stamp: return # Ignores old messages
        if message.vector.x == 0: return # Ignores invalid readings
        self.latest_position = message

    def register_scan(self, message):
        if not isinstance(message, LaserScan): return
        if self.ros_ctrl.Joy_active or self.switch:
            if self.moving:
                self.ros_ctrl.pub_vel.publish(Twist())
                self.moving = False
            return
        if not self.is_target_detected(): return
        self.publish_pid_control()

    def is_target_detected(self):
        if self.latest_position is None: return False
        two_seconds_ago = rospy.Time.now() - rospy.Duration(2)
        return self.latest_position.header.stamp > two_seconds_ago
    
    def dynamic_reconfigure_callback(self, config, level):
        self.switch = config['switch']
        self.laser_angle = config['laserAngle']
        self.priority_angle = config['priorityAngle']
        self.response_dist = config['ResponseDist']
        self.lin_pid.Set_pid(config['lin_Kp'], config['lin_Ki'], config['lin_Kd'])
        self.ang_pid.Set_pid(config['ang_Kp'], config['ang_Ki'], config['ang_Kd'])
        return config
    
    def publish_pid_control(self):
        if self.latest_position is None: return
        target_distance = self.latest_position.vector.x
        target_angle = self.latest_position.vector.y
        twist = Twist()
        twist.linear.x = -self.lin_pid.pid_compute(self.response_dist, target_distance)
        twist.angular.z = target_angle if target_angle > 0.02 else 0
        self.moving = True
        self.ros_ctrl.pub_vel.publish(twist)


if __name__ == '__main__':
    rospy.init_node('PIDController', anonymous=False)
    tracker = PIDController()
    rospy.spin()
