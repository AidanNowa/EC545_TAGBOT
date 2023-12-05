#!/usr/bin/env python
# coding:utf-8
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from opencv_apps.msg import Rect, RectArray


class ColorDetector:
    def __init__(self):
        rospy.on_shutdown(self.on_shutdown)
        self.bridge = CvBridge()
        self.sub_rgb = rospy.Subscriber('/camera/rgb/image_raw', Image, self.register_rgb_image, queue_size=1)
        self.pub_rect = rospy.Publisher('rect_detection', RectArray, queue_size=10)

    def on_shutdown(self):
        self.sub_rgb.unregister()
        self.pub_rect.unregister()
        rospy.loginfo("Shutting down this node.")

    def register_rgb_image(self, message):
        if not isinstance(message, Image): return
        print('ColorDetector register_rgb_image')
        image_frame = self.bridge.imgmsg_to_cv2(message, "bgr8")
        hsv_image_frame = cv2.cvtColor(image_frame, cv2.COLOR_BGR2HSV)
        rect_list = self.detect_colors(hsv_image_frame)
        self.publish_rect_detection(rect_list)
        
    def detect_colors(self, hsv_image_frame):
        red_lower = np.array([136, 87, 111], np.uint8) 
        red_upper = np.array([180, 255, 255], np.uint8) 
        red_mask = cv2.inRange(hsv_image_frame, red_lower, red_upper)
        kernel = np.ones((5, 5), "uint8") 
        red_mask = cv2.dilate(red_mask, kernel) 
        contours, hierarchy = cv2.findContours(red_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        print('red contours detected :', len(contours))
        rect_list = []
        for pic, contour in enumerate(contours):
            area = cv2.contourArea(contour)
            if area < 300: continue
            x, y, w, h = cv2.boundingRect(contour)
            rect = Rect(x, y, w, h)
            rect_list.append(rect)
        return rect_list
    
    def publish_rect_detection(self, rect_list):
        self.pub_rect.publish(rect_list)


if __name__ == '__main__':
    rospy.init_node('color_detector', anonymous=False)
    tracker = ColorDetector()
    rospy.spin()
