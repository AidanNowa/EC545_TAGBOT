#!/usr/bin/env python
# coding:utf-8
import time
import math
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from color_detector.msg import TargetPosition

CONTOUR_AREA_THRESHOLD = 300
DEPTH_ENCODING = '32FC1'
FOV_H = 73
IMAGE_WIDTH = 640
IMAGE_HEIGHT = 480


class ColorDetector:
    def __init__(self):
        rospy.on_shutdown(self.on_shutdown)
        self.bridge = CvBridge()
        self.target_list = []
        self.depth_image = None
        self.sub_rgb = rospy.Subscriber('/camera/rgb/image_raw', Image, self.register_rgb_image, queue_size=1)
        self.sub_depth = rospy.Subscriber('/camera/depth/image_raw', Image, self.register_depth_image, queue_size=1)
        # self.pub_position = rospy.Publisher('tagbot/target_position', TargetPosition, queue_size=10)

    def on_shutdown(self):
        cv2.destroyAllWindows()
        self.sub_rgb.unregister()
        self.sub_depth.unregister()
        # self.pub_position.unregister()

    def register_rgb_image(self, message):
        if not isinstance(message, Image): return
        image_frame = self.bridge.imgmsg_to_cv2(message, 'bgr8')
        hsv_image_frame = cv2.cvtColor(image_frame, cv2.COLOR_BGR2HSV)
        self.detect_colors(image_frame, hsv_image_frame)
        if self.depth_image is None: return
        target_position = self.get_target_position()
        self.publish_target_position(target_position)

    def register_depth_image(self, message):
        if not isinstance(message, Image): return
        image_frame = self.bridge.imgmsg_to_cv2(message, DEPTH_ENCODING)
        self.depth_image = cv2.resize(image_frame, (640, 480))
        
    def detect_colors(self, image_frame, hsv_image_frame):
        red_lower = np.array([136, 87, 111], np.uint8) 
        red_upper = np.array([180, 255, 255], np.uint8) 
        red_mask = cv2.inRange(hsv_image_frame, red_lower, red_upper)
        kernel = np.ones((5, 5), 'uint8')
        red_mask = cv2.dilate(red_mask, kernel) 
        im2, contours, hierarchy = cv2.findContours(red_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        rect_list = []
        for pic, contour in enumerate(contours):
            area = cv2.contourArea(contour)
            if area < CONTOUR_AREA_THRESHOLD: continue
            x, y, w, h = cv2.boundingRect(contour)
            rect = (x, y, w, h)
            rect_list.append(rect)
            image_frame = cv2.rectangle(image_frame, (x, y), (x + w, y + h), (255, 0, 0), 2) 
        # print('red contours detected :', len(rect_list))
        self.target_list = rect_list
        cv2.imshow('RED detection', image_frame)
        cv2.waitKey(10)
    
    def get_target_position(self):
        # Choose the closest point among the detected rects
        if len(self.target_list) <= 0: return None
        closest_target = None
        closest_distance = float('inf')
        for x, y, w, h in self.target_list:
            x_center = math.min(x + w / 2, IMAGE_WIDTH)
            y_center = math.min(y + h / 2, IMAGE_HEIGHT)
            distance = self.depth_image[x_center][y_center]
            if distance < closest_distance:
                closest_target = [x_center, y_center]
                closest_distance = distance
        if closest_target is None: return None
        x_center = closest_target[0]
        target_angle = x_center / IMAGE_WIDTH * FOV_H - FOV_H / 2
        print('closest_distance ', closest_distance)
        print('target_angle ', target_angle)
        pass
        # target_position = TargetPosition()
        # target_position.timestamp = time.time()
        # target_position.distance = closest_distance
        # target_position.angle = target_angle
        # return target_position

    def publish_target_position(self, target_position):
        if target_position is None: return
        print('Publish target_position: {}, {}, {}', target_position.timestamp, target_position.distance, target_position.angle)
        # self.pub_position.publish(target_position)


if __name__ == '__main__':
    rospy.init_node('color_detector', anonymous=False)
    tracker = ColorDetector()
    rospy.spin()
