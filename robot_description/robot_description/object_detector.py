# robot_description/robot_description/object_detector.py
# Description: A simple object detector node that subscribes to the camera image topic and detects blue objects in the image.

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
from std_msgs.msg import String


class ObjectDetector(Node):
    def __init__(self):
        super().__init__('object_detector')
        self.get_logger().info("Object detector node started")
        self.subscription = self.create_subscription(
            Image,
            '/excavator/camera/image_raw',
            self.image_callback,
            10)
        self.bridge = CvBridge()
        self.detected_object_publisher_ = self.create_publisher(
            String, 'detected_objects', 10)

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            self.get_logger().error(f"CvBridge Error: {e}")
            return

        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        # Detect red objects (transport_box)
        lower_red1 = np.array([0, 100, 100])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([170, 100, 100])
        upper_red2 = np.array([180, 255, 255])
        mask1 = cv2.inRange(hsv_image, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv_image, lower_red2, upper_red2)
        mask = cv2.bitwise_or(mask1, mask2)
        contours, _ = cv2.findContours(
            mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        detected_objects_msg = String()
        detected_objects_str = ""

        for contour in contours:
            area = cv2.contourArea(contour)
            if area > 100:  # Filter noise
                M = cv2.moments(contour)
                if M["m00"] != 0:
                    cX = int(M["m10"] / M["m00"])
                    cY = int(M["m01"] / M["m00"])
                    label = "Transport Box" if 500 < area < 1500 else "Large Object"
                    cv2.drawContours(cv_image, [contour], -1, (0, 255, 0), 2)
                    cv2.circle(cv_image, (cX, cY), 7, (255, 255, 255), -1)
                    cv2.putText(cv_image, label, (cX - 20, cY - 20),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
                    detected_objects_str += f"{label} at ({cX}, {cY}); "

        if detected_objects_str:
            detected_objects_msg.data = detected_objects_str
            self.get_logger().info(f"Detected: {detected_objects_str}")
        else:
            detected_objects_msg.data = "No objects detected"
            self.get_logger().info("No objects detected")
        self.detected_object_publisher_.publish(detected_objects_msg)

        cv2.imshow("Object Detection", cv_image)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    object_detector = ObjectDetector()
    rclpy.spin(object_detector)
    object_detector.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
