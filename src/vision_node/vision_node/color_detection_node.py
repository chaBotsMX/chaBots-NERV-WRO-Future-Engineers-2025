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
        
        # Parámetros HSV para verde (SIN CAMBIOS)
        self.lower_green = np.array([37, 143, 0])
        self.upper_green = np.array([68, 255, 193])
        
        # Parámetros HSV para rojo (PARCHE: dos rangos y se unirán)
        # Rango bajo (cerca de 0)
        self.lower_red1 = np.array([0,   100, 80])
        self.upper_red1 = np.array([2,  255, 255])
        # Rango alto (cerca de 179)
        self.lower_red2 = np.array([170, 100, 80])
        self.upper_red2 = np.array([179, 255, 255])
        
        # Parámetros de la cámara
        self.FOCAL_LENGTH = 1131   # píxeles
        self.KNOWN_WIDTH = 14.0    # cms
        self.FRAME_WIDTH = 1920
        
        self.get_logger().info("Object Tracker Node iniciado - Detectando verde y rojo")
        cv2.namedWindow("Frame", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("Frame", 960, 540)  # ajusta a tu pantalla

    def detect_objects(self, hsv):
        """
        Detecta objetos verdes y rojos en la imagen HSV
        Retorna una lista de objetos detectados con su información
        """
        detected_objects = []
        
        # Detectar objetos verdes (SIN CAMBIOS)
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
        
        # Detectar objetos rojos (PARCHE: combinar dos rangos)
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

        return detected_objects

    def image_callback(self, msg):
        # Convertir imagen ROS -> OpenCV
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        # Detectar objetos de ambos colores
        detected_objects = self.detect_objects(hsv)
        
        find = float(0)
        
        # Si hay objetos detectados, encontrar el de mayor área
        if detected_objects:
            # Ordenar por área de mayor a menor y tomar el primero
            largest_object = max(detected_objects, key=lambda obj: obj['area'])
            
            x, y, w, h = largest_object['bbox']
            cx, cy = x + w // 2, y + h // 2
            
            # Calcular distancia
            distance = (self.KNOWN_WIDTH * self.FOCAL_LENGTH) / w
            
            # Calcular ángulo
            dx = cx - self.FRAME_WIDTH // 2
            angle = np.degrees(np.arctan(dx / self.FOCAL_LENGTH))
            
            # Publicar datos solo del objeto de mayor área
            self.pub_distance.publish(Float32(data=float(distance)))
            self.pub_angle.publish(Float32(data=float(angle)))
            self.pub_color.publish(Float32(data=float(largest_object['color_code'])))
            find = float(1)
            
            # Dibujar rectángulo y texto solo para el objeto de mayor área
            cv2.rectangle(frame, (x, y), (x + w, y + h), largest_object['bgr_color'], 2)
            
            # Texto con información del objeto
            cv2.putText(frame, f"{largest_object['color'].capitalize()}", (x, y - 60),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, largest_object['bgr_color'], 2)
            cv2.putText(frame, f"Dist: {distance:.1f} cm", (x, y - 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
            cv2.putText(frame, f"Ang: {angle:.1f} deg", (x, y - 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
            
        self.pub_status.publish(Float32(data=find))
        
        # Mostrar ventanas (para debugs)
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
