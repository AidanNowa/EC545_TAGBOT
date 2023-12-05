#!/usr/bin/env python
# coding:utf-8
import time
import rospy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from opencv_apps.msg import RectArray, RectArrayStamped


class HumanDetector:
    def __init__(self) -> None:
        self.bridge = CvBridge()
        self.hog = cv2.HOGDescriptor()
        self.hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())
        self.sub_rgb = rospy.Subscriber("/camera/rgb/image_raw", Image, self.register_rgb_images)
        self.pub_detection = rospy.Publisher("human_detection", RectArrayStamped, queue_size=10)

    def register_rgb_images(self, message) -> None:
        if not isinstance(message, Image): return
        frame = self.bridge.imgmsg_to_cv2(message, "bgr8")
        detected_humans = self._detect_humans(frame)
        print('human detected :', len(detected_humans))
        if len(detected_humans) <= 0: return
        detection_message = self._create_detection_message(detected_humans)
        self.pub_detection.publish(detection_message)

    def _detect_humans(self, image) -> RectArray:
        # RectArray = [Rect] = [(x, y, width, height)]
        humans, _ = self.hog.detectMultiScale(image, winStride=(10, 10), padding=(32, 32), scale=1.1)
        return humans
    
    def _create_detection_message(self, humans: RectArray) -> RectArrayStamped:
        return RectArrayStamped(time.time(), humans)


if __name__ == '__main__':
    rospy.init_node('human_detector', anonymous=False)
    detector = HumanDetector()
    rospy.spin()
