#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String, Float32, Float32MultiArray, MultiArrayDimension
from cv_bridge import CvBridge
import cv2
import numpy as np

class ObjectTracker(Node):
    def __init__(self):
        super().__init__("object_tracker")

        # Obtener imágenes de la cámara utilizando el tópico "/camera/image_raw"
        self.sub = self.create_subscription(
            Image, "/camera/image_raw", self.image_callback, 10)

        # Publishers para la información de los objetos
        self.pub_objects = self.create_publisher(Float32MultiArray, "/objects/detections", 10)
        self.pub_status = self.create_publisher(Float32, "/object/status", 10)

        self.bridge = CvBridge()

        # Rangos HSV para los colores verde, rojo y morado
        self.lower_green = np.array([38, 68, 50])
        self.upper_green = np.array([80, 255, 185])

        self.lower_red1 = np.array([0, 150, 100])
        self.upper_red1 = np.array([15, 255, 255])

        self.lower_red2 = np.array([152, 150, 100])
        self.upper_red2 = np.array([180, 255, 255])
 
        self.lower_purple = np.array([119, 43, 150])
        self.upper_purple = np.array([166, 255, 255])

        self.lower_blue = np.array([110,32,79])
        self.upper_blue = np.array([133,255,154])

        # Parámetros de la cámara necesarios para calcular distancia y ángulo
        self.FOCAL_LENGTH = 1131   # píxeles
        self.KNOWN_WIDTH = 14.0    # cms
        self.FRAME_WIDTH = 1920

        self.get_logger().info("Object Tracker Node iniciado - Formato: [color, distance, angle] por objeto")
        cv2.namedWindow("Frame", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("Frame", 960, 540)

    def detect_objects(self, hsv):
        """
        Detecta objetos verdes, rojos y morados en la imagen HSV
        """
        detected_objects = []

        # Detección de objetos verdes
        mask_green = cv2.inRange(hsv, self.lower_green, self.upper_green)
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5,5))
        mask_green = cv2.morphologyEx(mask_green, cv2.MORPH_CLOSE, kernel)
        mask_green = cv2.morphologyEx(mask_green, cv2.MORPH_OPEN, kernel)
        mask_green = cv2.dilate(mask_green, kernel, iterations=2)
        contours_green, _ = cv2.findContours(mask_green, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for cnt in contours_green:
            area = cv2.contourArea(cnt)
            if area > 4000:
                x, y, w, h = cv2.boundingRect(cnt)
                detected_objects.append({
                    'contour': cnt,
                    'bbox': (x, y, w, h),
                    'color': 'green',
                    'color_code': 0,
                    'bgr_color': (0, 255, 0),
                    'area': area
                })

        # Detección de objetos rojos
        mask_r1 = cv2.inRange(hsv, self.lower_red1, self.upper_red1)
        mask_r2 = cv2.inRange(hsv, self.lower_red2, self.upper_red2)
        mask_red = cv2.bitwise_or(mask_r1, mask_r2)
        mask_red = cv2.morphologyEx(mask_red, cv2.MORPH_CLOSE, kernel)
        mask_red = cv2.morphologyEx(mask_red, cv2.MORPH_OPEN, kernel)
        mask_red = cv2.dilate(mask_red, kernel, iterations=2)
        contours_red, _ = cv2.findContours(mask_red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for cnt in contours_red:
            area = cv2.contourArea(cnt)
            if area > 6000:
                x, y, w, h = cv2.boundingRect(cnt)
                detected_objects.append({
                    'contour': cnt,
                    'bbox': (x, y, w, h),
                    'color': 'red',
                    'color_code': 1,
                    'bgr_color': (0, 0, 255),
                    'area': area
                })

        # Detección de objetos morados
        mask_purple = cv2.inRange(hsv, self.lower_purple, self.upper_purple)
        mask_purple = cv2.morphologyEx(mask_purple, cv2.MORPH_CLOSE, kernel)
        mask_purple = cv2.morphologyEx(mask_purple, cv2.MORPH_OPEN, kernel)
        mask_purple = cv2.dilate(mask_purple, kernel, iterations=2)
        contours_purple, _ = cv2.findContours(mask_purple, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for cnt in contours_purple:
            area = cv2.contourArea(cnt)
            if area > 4000:
                x, y, w, h = cv2.boundingRect(cnt)
                detected_objects.append({
                    'contour': cnt,
                    'bbox': (x, y, w, h),
                    'color': 'purple',
                    'color_code': 2,
                    'bgr_color': (128, 0, 128),
                    'area': area
                })


        return detected_objects

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        x, y, w, h = 0, 200, 1920, 900
        H, W = frame.shape[:2]
        x = max(0, min(x, W - 1))
        y = max(0, min(y, H - 1))
        w = min(w, W - x)
        h = min(h, H - y)

        frame = frame[y:y+h, x:x+w]

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        detected_objects = self.detect_objects(hsv)
        find = float(len(detected_objects))

        self.pub_status.publish(Float32(data=find))

        objects_msg = Float32MultiArray()

        # Cada objeto tiene: [color_code, distance, angle]
        dim_objects = MultiArrayDimension()
        dim_objects.label = "objects"
        dim_objects.size = len(detected_objects)
        dim_objects.stride = len(detected_objects) * 3

        dim_properties = MultiArrayDimension()
        dim_properties.label = "properties"  # [color, distance, angle]
        dim_properties.size = 3
        dim_properties.stride = 3

        objects_msg.layout.dim = [dim_objects, dim_properties]
        objects_msg.layout.data_offset = 0

        objects_data = []

        for i, obj in enumerate(detected_objects):
            x, y, w, h = obj['bbox']
            cx, cy = x + w // 2, y + h // 2

            distance = (self.KNOWN_WIDTH * self.FOCAL_LENGTH) / w
            dx = cx - self.FRAME_WIDTH // 2
            angle = np.degrees(np.arctan(dx / self.FOCAL_LENGTH))

            # Agregar datos del objeto: [color_code, distance, angle]
            objects_data.extend([
                float(obj['color_code']),  # 0=verde, 1=rojo, 2=morado
                float(distance),
                float(angle)
            ])

            cv2.rectangle(frame, (x, y), (x + w, y + h), obj['bgr_color'], 2)
            cv2.putText(frame, f"{obj['color'].capitalize()} #{i}", (x, y - 60),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, obj['bgr_color'], 2)
            cv2.putText(frame, f"Dist: {distance:.1f} cm", (x, y - 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
            cv2.putText(frame, f"Ang: {angle:.1f} deg", (x, y - 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

        objects_msg.data = objects_data
        self.pub_objects.publish(objects_msg)

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