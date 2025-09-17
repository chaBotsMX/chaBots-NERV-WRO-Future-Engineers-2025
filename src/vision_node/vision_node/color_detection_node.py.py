#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np


def nothing(x):
    pass


class HSVTuner(Node):
    def __init__(self):
        super().__init__('hsv_tuner')
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )
        self.bridge = CvBridge()

        # Crear la ventana y las trackbars
        cv2.namedWindow("Trackbars")
        cv2.createTrackbar("H_min", "Trackbars", 0, 179, nothing)
        cv2.createTrackbar("H_max", "Trackbars", 179, 179, nothing)
        cv2.createTrackbar("S_min", "Trackbars", 0, 255, nothing)
        cv2.createTrackbar("S_max", "Trackbars", 255, 255, nothing)
        cv2.createTrackbar("V_min", "Trackbars", 0, 255, nothing)
        cv2.createTrackbar("V_max", "Trackbars", 255, 255, nothing)

    def image_callback(self, msg):
        # Convertir ROS Image a OpenCV
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Obtener valores de las trackbars
        h_min = cv2.getTrackbarPos("H_min", "Trackbars")
        h_max = cv2.getTrackbarPos("H_max", "Trackbars")
        s_min = cv2.getTrackbarPos("S_min", "Trackbars")
        s_max = cv2.getTrackbarPos("S_max", "Trackbars")
        v_min = cv2.getTrackbarPos("V_min", "Trackbars")
        v_max = cv2.getTrackbarPos("V_max", "Trackbars")

        lower = np.array([h_min, s_min, v_min])
        upper = np.array([h_max, s_max, v_max])

        # Crear máscara
        mask = cv2.inRange(hsv, lower, upper)
        result = cv2.bitwise_and(frame, frame, mask=mask)

        # Mostrar resultados
        cv2.imshow("Frame", frame)
        cv2.imshow("Mask", mask)
        cv2.imshow("Result", result)

        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = HSVTuner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
