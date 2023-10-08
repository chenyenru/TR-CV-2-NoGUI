import rclpy
import cv2 as cv
import numpy as np
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, Empty, String

bridge = CvBridge()


def image_centroid(image):
    M = cv.moments(image)

    # calculate x,y coordinate of center
    if M["m00"] != 0:
        cX = int(M["m10"] / M["m00"])
        cY = int(M["m01"] / M["m00"])
    else:
        cX, cY = 0, 0
    return np.array([cX, cY])


def filter_hsv(hsvimage, color, dist):
    hue = hsvimage[:, :, 0]
    sat = hsvimage[:, :, 1]
    val = hsvimage[:, :, 2]
    h, s, v = color
    centering = 90 - h
    hue += centering
    hue = np.mod(hue, 180)
    lower = np.array([90 - dist, 100, 100])
    upper = np.array([90 + dist, 255, 255])
    adjimg = np.dstack((hue, sat, val))
    mask = cv.inRange(adjimg, lower, upper)
    return mask


class Solution(Node):
    def __init__(self):
        super().__init__("solution")
        self.anglepub = self.create_publisher(Float32, "/desired_angle", 10)
        self.pubtimer = self.create_timer(1 / 30, self.timer_callback)

        # Subscriptions
        self.anglesub = self.create_subscription(
            Float32, "/current_angle", self.angle_sub_callback, 10
        )

        self.imagesub = self.create_subscription(
            Image, "robotcam", self.image_pub_callback, 10
        )

        self.center = 320
        self.dist = 5

    def angle_sub_callback(self, message: Float32):
        pass

    def image_pub_callback(self, message: Image):
        angle_to_red = None
        TARGET_COLOR = [0, 255, 153]

        if Image != None:
            hsvimage = bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
            red_mask = filter_hsv(hsvimage, [0, 255, 153], 20)

        if cv.countNonZero(red_mask) > 0:
            x, y = image_centroid(red_mask)
            # Only change camera angle when red cube is not within this range
            if not (self.center - self.dist < x < self.center + self.dist):
                angle_to_red = x - self.center / float(self.center)
                # self.tm.set_joint_angle(angle_to_red)

        angle = Float32()
        if angle_to_red != None:
            angle.data = angle_to_red
        else:
            angle.data = 0.0

        # print(f"Angle is {angle.data}")
        self.anglepub.publish(angle)

    def timer_callback(self):
        pass


def main(args=None):
    rclpy.init(args=args)
    sol = Solution()

    while True:
        rclpy.spin(sol)
