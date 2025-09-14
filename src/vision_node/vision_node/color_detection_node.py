#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String, Float32, Float32MultiArray
from cv_bridge import CvBridge
import cv2
import numpy as np

class ObjectTracker(Node):
    def __init__(self):
        super().__init__("object_tracker")

        # Obtener imágenes de la cámara utilizando el tópico "/camera/image_raw"
        self.sub = self.create_subscription(
            Image, "/camera/image_raw", self.image_callback, 10)

        # Publishers para la información de los objetos y el estado
        self.pub_objects = self.create_publisher(Float32MultiArray, "/objects/info", 10)
        self.pub_status = self.create_publisher(Float32, "/object/status", 10)

        self.bridge = CvBridge()

        # Rangos HSV para los colores verde, rojo y morado
        self.lower_green = np.array([37, 143, 0])
        self.upper_green = np.array([68, 255, 193])

        self.lower_red1 = np.array([0,   100, 80])
        self.upper_red1 = np.array([2,  255, 255])

        self.lower_red2 = np.array([170, 100, 80])
        self.upper_red2 = np.array([179, 255, 255])

        self.lower_purple = np.array([130, 50, 50])
        self.upper_purple = np.array([160, 255, 255])

        # Parámetros de la cámara necesarios para calcular distancia y ángulo
        self.FOCAL_LENGTH = 1131   # píxeles
        self.KNOWN_WIDTH = 14.0    # cms
        self.FRAME_WIDTH = 1920

        self.get_logger().info("Object Tracker Node iniciado - Detectando verde, rojo y morado")
        cv2.namedWindow("Frame", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("Frame", 960, 540)  # ajusta a tu pantalla

    def detect_objects(self, hsv):
        """
        Detecta objetos verdes y rojos en la imagen HSV
        Retorna una lista de objetos detectados con su información
        """
        detected_objects = []

        mask_green = cv2.inRange(hsv, self.lower_green, self.upper_green)
        contours_green, _ = cv2.findContours(mask_green, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for cnt in contours_green:
            area = cv2.contourArea(cnt)
            if area > 4000:
                x, y, w, h = cv2.boundingRect(cnt)
                detected_objects.append({
                    'contour': cnt,
                    'bbox': (x, y, w, h),
                    'color': 'green',
                    'color_code': 0,  # 0 para verde
                    'bgr_color': (0, 255, 0),
                    'area': area
                })

        mask_r1 = cv2.inRange(hsv, self.lower_red1, self.upper_red1)
        mask_r2 = cv2.inRange(hsv, self.lower_red2, self.upper_red2)
        mask_red = cv2.bitwise_or(mask_r1, mask_r2)

        contours_red, _ = cv2.findContours(mask_red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for cnt in contours_red:
            area = cv2.contourArea(cnt)
            if area > 4000:
                x, y, w, h = cv2.boundingRect(cnt)
                detected_objects.append({
                    'contour': cnt,
                    'bbox': (x, y, w, h),
                    'color': 'red',
                    'color_code': 1,  # 1 para rojo
                    'bgr_color': (0, 0, 255),
                    'area': area
                })

        mask_purple = cv2.inRange(hsv, self.lower_purple, self.upper_purple)
        contours_purple, _ = cv2.findContours(mask_purple, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for cnt in contours_purple:
            area = cv2.contourArea(cnt)
            if area > 4000:
                x, y, w, h = cv2.boundingRect(cnt)
                detected_objects.append({
                    'contour': cnt,
                    'bbox': (x, y, w, h),
                    'color': 'purple',
                    'color_code': 2,  # 2 para morado
                    'bgr_color': (128, 0, 128),
                    'area': area
                })

        return detected_objects

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        detected_objects = self.detect_objects(hsv)

        find = float(len(detected_objects))

        objects_data = []

        for obj in detected_objects:
            x, y, w, h = obj['bbox']
            cx, cy = x + w // 2, y + h // 2

            distance = (self.KNOWN_WIDTH * self.FOCAL_LENGTH) / w

            dx = cx - self.FRAME_WIDTH // 2
            angle = np.degrees(np.arctan(dx / self.FOCAL_LENGTH))

            objects_data.extend([
                float(cx),
                float(cy),
                float(distance),
                float(angle),
                float(obj['color_code']),
                float(obj['area'])
            ])

            cv2.rectangle(frame, (x, y), (x + w, y + h), obj['bgr_color'], 2)

            cv2.putText(frame, f"{obj['color'].capitalize()}", (x, y - 60),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, obj['bgr_color'], 2)
            cv2.putText(frame, f"Dist: {distance:.1f} cm", (x, y - 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
            cv2.putText(frame, f"Ang: {angle:.1f} deg", (x, y - 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

        objects_msg = Float32MultiArray()
        objects_msg.data = objects_data
        self.pub_objects.publish(objects_msg)

        self.pub_status.publish(Float32(data=find))

        cv2.imshow("Frame", frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = ObjectTracker()
    rclpy.spin(node)
    node.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
