#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from std_msgs.msg import String, Float32
import cv2
from cv_bridge import CvBridge
import numpy as np
import math

class ColorDetectionNode(Node):
    def __init__(self):
        super().__init__('color_detection_node')
        
        # Bridge para convertir entre ROS2 y OpenCV
        self.bridge = CvBridge()
        
        # Suscriptor a la imagen de la cámara
        self.image_subscription = self.create_subscription(
            Image,
            '/camera/image_raw',  # Cambiar según tu tópico de cámara
            self.image_callback,
            10
        )
        
        # Publishers para la información detectada
        self.color_publisher = self.create_publisher(String, '/detected_color', 10)
        self.distance_publisher = self.create_publisher(Float32, '/object_distance', 10)
        self.angle_publisher = self.create_publisher(Float32, '/object_angle', 10)
        self.position_publisher = self.create_publisher(Point, '/object_position', 10)
        
        # Rangos HSV para detección de colores
        # Verde
        self.green_lower = np.array([40, 50, 50])
        self.green_upper = np.array([80, 255, 255])
        
        # Rojo (el rojo tiene dos rangos en HSV)
        self.red_lower1 = np.array([0, 50, 50])
        self.red_upper1 = np.array([10, 255, 255])
        self.red_lower2 = np.array([170, 50, 50])
        self.red_upper2 = np.array([180, 255, 255])
        
        # Parámetros de la cámara (ajustar según tu configuración)
        self.image_width = 640
        self.image_height = 480
        self.focal_length = 500  # Longitud focal aproximada en píxeles
        self.object_real_size = 0.05  # Tamaño real conocido del objeto en metros
        
        self.get_logger().info('Nodo de detección de colores iniciado')

    def image_callback(self, msg):
        try:
            # Convertir imagen ROS a formato OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            self.image_width = cv_image.shape[1]
            self.image_height = cv_image.shape[0]
            
            # Procesar la imagen
            self.process_image(cv_image)
            
        except Exception as e:
            self.get_logger().error(f'Error procesando imagen: {str(e)}')

    def process_image(self, image):
        # Convertir BGR a HSV
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        # Detectar objetos verdes
        green_mask = cv2.inRange(hsv, self.green_lower, self.green_upper)
        green_detected = self.detect_objects(green_mask, 'green', image)
        
        # Detectar objetos rojos (combinando ambos rangos)
        red_mask1 = cv2.inRange(hsv, self.red_lower1, self.red_upper1)
        red_mask2 = cv2.inRange(hsv, self.red_lower2, self.red_upper2)
        red_mask = cv2.bitwise_or(red_mask1, red_mask2)
        red_detected = self.detect_objects(red_mask, 'red', image)
        
        # Si no se detecta ningún objeto, publicar valores por defecto
        if not green_detected and not red_detected:
            self.publish_detection('none', 0.0, 0.0, Point())

    def detect_objects(self, mask, color, original_image):
        # Aplicar operaciones morfológicas para limpiar la máscara
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        
        # Encontrar contornos
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if len(contours) == 0:
            return False
        
        # Encontrar el contorno más grande (objeto más prominente)
        largest_contour = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(largest_contour)
        
        # Filtrar contornos muy pequeños (ruido)
        if area < 500:
            return False
        
        # Calcular el centroide del objeto
        M = cv2.moments(largest_contour)
        if M["m00"] != 0:
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])
        else:
            return False
        
        # Calcular distancia estimada basada en el área
        distance = self.calculate_distance(area)
        
        # Calcular ángulo basado en la posición horizontal
        angle = self.calculate_angle(cx)
        
        # Crear mensaje de posición
        position = Point()
        position.x = float(cx)
        position.y = float(cy)
        position.z = distance
        
        # Publicar información detectada
        self.publish_detection(color, distance, angle, position)
        
        # Dibujar información en la imagen (opcional, para debug)
        cv2.circle(original_image, (cx, cy), 10, (255, 255, 255), -1)
        cv2.drawContours(original_image, [largest_contour], -1, (0, 255, 0), 2)
        cv2.putText(original_image, f'{color}: {distance:.2f}m, {angle:.1f}deg', 
                   (cx-50, cy-20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        
        # Mostrar imagen procesada (opcional, comentar en producción)
        cv2.imshow('Color Detection', original_image)
        cv2.waitKey(1)
        
        return True

    def calculate_distance(self, area):
        """
        Calcular distancia estimada basada en el área del objeto detectado
        Asume que conocemos el tamaño real del objeto
        """
        if area <= 0:
            return 0.0
        
        # Fórmula: distancia = (tamaño_real * longitud_focal) / sqrt(área)
        # Esta es una aproximación simple, se puede mejorar con calibración
        distance = (self.object_real_size * self.focal_length) / math.sqrt(area)
        
        # Limitar distancia a un rango razonable
        distance = max(0.1, min(distance, 10.0))
        
        return distance

    def calculate_angle(self, x_pixel):
        """
        Calcular ángulo basado en la posición horizontal del objeto
        Ángulo 0 = centro, negativo = izquierda, positivo = derecha
        """
        # Centro de la imagen
        center_x = self.image_width / 2
        
        # Ángulo en radianes basado en la posición del pixel
        # Asumiendo un campo de visión horizontal de ~60 grados
        fov_horizontal = 60.0  # grados
        pixel_angle = (x_pixel - center_x) / center_x * (fov_horizontal / 2)
        
        return pixel_angle

    def publish_detection(self, color, distance, angle, position):
        """
        Publicar información de detección en los tópicos correspondientes
        """
        # Publicar color detectado
        color_msg = String()
        color_msg.data = color
        self.color_publisher.publish(color_msg)
        
        # Publicar distancia
        distance_msg = Float32()
        distance_msg.data = distance
        self.distance_publisher.publish(distance_msg)
        
        # Publicar ángulo
        angle_msg = Float32()
        angle_msg.data = angle
        self.angle_publisher.publish(angle_msg)
        
        # Publicar posición completa
        self.position_publisher.publish(position)
        
        # Log de información detectada
        if color != 'none':
            self.get_logger().info(
                f'Detectado: Color={color}, Distancia={distance:.2f}m, Ángulo={angle:.1f}°'
            )

def main(args=None):
    rclpy.init(args=args)
    
    color_detection_node = ColorDetectionNode()
    
    try:
        rclpy.spin(color_detection_node)
    except KeyboardInterrupt:
        pass
    finally:
        color_detection_node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()

if __name__ == '__main__':
    main()