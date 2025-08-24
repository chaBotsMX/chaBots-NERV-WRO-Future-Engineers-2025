#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String, Float32
from cv_bridge import CvBridge
import cv2
import numpy as np

class ObjectTracker(Node):
    def __init__(self):
        super().__init__("object_tracker")

        # Suscriptor de la cámara
        self.sub = self.create_subscription(
            Image, "/camera/image_raw", self.image_callback, 10)

        # Publicadores
        self.pub_distance = self.create_publisher(Float32, "/object/distance", 10)
        self.pub_angle = self.create_publisher(Float32, "/object/angle", 10)
        self.pub_color = self.create_publisher(Float32, "/object/color", 10)
        self.pub_status = self.create_publisher(Float32, "/object/status", 10)

        self.bridge = CvBridge()

        # Parámetros HSV (ajusta según tu máscara)
        self.lower_green = np.array([37, 143, 0])
        self.upper_green = np.array([68, 255, 181])

        # Parámetros de la cámara (ajusta según tu setup real)
        self.FOCAL_LENGTH = 600   # píxeles
        self.KNOWN_WIDTH = 14.0   # cm
        self.FRAME_WIDTH = 640

        self.get_logger().info("Object Tracker Node iniciado")

    def image_callback(self, msg):
        # Convertir imagen ROS -> OpenCV
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.lower_green, self.upper_green)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        find = float(0)
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > 500:
                x, y, w, h = cv2.boundingRect(cnt)
                cx, cy = x + w // 2, y + h // 2

                # Calcular distancia
                distance = (self.KNOWN_WIDTH * self.FOCAL_LENGTH) / w

                # Calcular ángulo
                dx = cx - self.FRAME_WIDTH // 2
                angle = np.degrees(np.arctan(dx / self.FOCAL_LENGTH))

                # Publicar
                self.pub_distance.publish(Float32(data=float(distance)))
                self.pub_angle.publish(Float32(data=float(angle)))
                self.pub_color.publish(Float32(data=float(0)))
                find = float(1)

                # Dibujar en la ventana
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                cv2.putText(frame, f"Dist: {distance:.1f} cm", (x, y - 20),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
                cv2.putText(frame, f"Ang: {angle:.1f} deg", (x, y - 40),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        
        self.pub_status.publish(Float32(data=find))

        # Mostrar ventanas (para debug)
        #cv2.imshow("Frame", frame)
        #cv2.imshow("Mask", mask)
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
