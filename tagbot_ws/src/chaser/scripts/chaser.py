import rospy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from opencv_apps import RectArrayStamped


class Chaser:
    def __init__(self) -> None:
        rospy.on_shutdown(self.cancel)
        self.bridge = CvBridge()
        self.sub_rgb = rospy.Subscriber('/camera/rgb/image_raw', Image, self.register_rgb_image)
        self.sub_depth = rospy.Subscriber('/camera/depth/image_raw', Image, self.register_depth_image)
        self.sub_people_detector = rospy.Subscriber('/people_detect/found', RectArrayStamped, self.register_people_detector)
        self.is_target_detected = False
        self.latest_rgb_image = None
        self.latest_depth_image = None

    def cancel(self):
        pass

    def register_rgb_image(self, message):
        if not isinstance(message, Image): return
        frame = self.bridge.imgmsg_to_cv2(message, 'bgr8')
        # Standardize the input image size
        frame = cv2.resize(frame, (640, 480))
        cv2.imshow("color_image", frame)
        cv2.waitKey(10)

    def register_depth_image(self, message):
        if not isinstance(message, Image): return
        depth_frame = self.bridge.imgmsg_to_cv2(message, desired_encoding='32FC1')

    def register_people_detector(self, message):
        if not isinstance(message, RectArrayStamped): return
        print('rects ', message.rects)


if __name__ == '__main__':
    rospy.init_node('chaser', anonymous=False)
    tracker = Chaser()
    rospy.spin()
