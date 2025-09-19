#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class HSVCalibrator(Node):
    def __init__(self):
        super().__init__("hsv_calibrator")
        
        # Suscriptor de la cámara
        self.subscription = self.create_subscription(
            Image,
            "/camera/image_raw",  # Cambia este tópico según tu configuración
            self.image_callback,
            10
        )
        
        self.bridge = CvBridge()
        self.current_frame = None
        
        self.dilate_k = 3
        self.dilate_iterations = 2

        # Valores iniciales para los sliders
        self.h_min = 0
        self.s_min = 50
        self.v_min = 50
        self.h_max = 180
        self.s_max = 255
        self.v_max = 255
        
        # Crear ventanas
        cv2.namedWindow('Original', cv2.WINDOW_AUTOSIZE)
        cv2.namedWindow('Mask', cv2.WINDOW_AUTOSIZE)
        cv2.namedWindow('Result', cv2.WINDOW_AUTOSIZE)
        cv2.namedWindow('HSV Controls', cv2.WINDOW_AUTOSIZE)
        
        # Crear sliders para HSV
        cv2.createTrackbar('H Min', 'HSV Controls', self.h_min, 180, self.update_h_min)
        cv2.createTrackbar('S Min', 'HSV Controls', self.s_min, 255, self.update_s_min)
        cv2.createTrackbar('V Min', 'HSV Controls', self.v_min, 255, self.update_v_min)
        cv2.createTrackbar('H Max', 'HSV Controls', self.h_max, 180, self.update_h_max)
        cv2.createTrackbar('S Max', 'HSV Controls', self.s_max, 255, self.update_s_max)
        cv2.createTrackbar('V Max', 'HSV Controls', self.v_max, 255, self.update_v_max)
        
        cv2.createTrackbar('Dilate Kernel', 'HSV Controls', self.dilate_k, 20, lambda x: None)
        cv2.createTrackbar('Dilate Iterations', 'HSV Controls', self.dilate_iterations, 10, lambda x: None)

        # Crear slider para seleccionar color predefinido
        cv2.createTrackbar('Preset', 'HSV Controls', 0, 4, self.load_preset)
        
        self.get_logger().info("=== CALIBRADOR HSV ROS ===")
        self.get_logger().info("Suscrito a: /camera/image_raw")
        self.get_logger().info("Controles:")
        self.get_logger().info("- Ajusta los sliders para calibrar los rangos HSV")
        self.get_logger().info("- Preset 0: Manual, 1: Verde, 2: Rojo1, 3: Rojo2, 4: Azul")
        self.get_logger().info("- Presiona 's' para guardar los valores actuales")
        self.get_logger().info("- Presiona 'r' para resetear a valores por defecto")
        self.get_logger().info("- Presiona 'q' para salir")
        self.get_logger().info("- Presiona 'p' para imprimir valores actuales")
        
    def image_callback(self, msg):
        """Callback que recibe las imágenes de ROS"""
        try:
            # Convertir imagen ROS -> OpenCV
            self.current_frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.process_frame()
        except Exception as e:
            self.get_logger().error(f"Error procesando imagen: {e}")
    
    def process_frame(self):
        """Procesa el frame actual con los valores HSV"""
        if self.current_frame is None:
            return
        frame = self.current_frame.copy()        
        x, y,w, h = 0,200,1920, 740
        H, W = frame.shape[:2]
        x = max(0, min(x, W - 1))
        y = max(0, min(y, H - 1))
        w = max(1, min(w, W - x))
        h = max(1, min(h, H - y))

        frame = self.current_frame[y:y+h, x:x+w]
        
        # Redimensionar frame si es muy grande
        height, width = frame.shape[:2]
        if width > 800:
            scale = 800 / width
            new_width = int(width * scale)
            new_height = int(height * scale)
            frame = cv2.resize(frame, (new_width, new_height))
        

        frame = cv2.GaussianBlur(frame, (5, 5), 0)
        # Convertir a HSV
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        # Crear máscara con los valores actuales
        lower_bound = np.array([self.h_min, self.s_min, self.v_min])
        upper_bound = np.array([self.h_max, self.s_max, self.v_max])
        mask = cv2.inRange(hsv, lower_bound, upper_bound)
        

        k = max(1, self.dilate_k)  # Kernel no puede ser 0
        self.dilate_k = k
        self.dilate_iterations = max(1, self.dilate_iterations)

        # Aplicar dilatación
        k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (self.dilate_k, self.dilate_k))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k)
        mask = cv2.dilate(mask, k, iterations=self.dilate_iterations)

        # Aplicar máscara
        result = cv2.bitwise_and(frame, frame, mask=mask)
        
        # Encontrar contornos para mostrar información
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        # Dibujar información en el frame original
        info_frame = frame.copy()
        objects_found = 0
        total_area = 0
        
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > 500:  # Filtrar objetos pequeños
                x, y, w, h = cv2.boundingRect(cnt)
                cv2.rectangle(info_frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                cv2.putText(info_frame, f"Area: {int(area)}", (x, y-10),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
                objects_found += 1
                total_area += area
        
        # Mostrar información en pantalla
        cv2.putText(info_frame, f"Objetos: {objects_found} | Area total: {int(total_area)}", (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv2.putText(info_frame, f"HSV: [{self.h_min},{self.s_min},{self.v_min}]-[{self.h_max},{self.s_max},{self.v_max}]", 
                   (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        
        # Agregar información del tópico ROS
        cv2.putText(info_frame, "Fuente: ROS /camera/image_raw", (10, frame.shape[0] - 20),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
        
        # Mostrar ventanas
        cv2.imshow('Original', info_frame)
        cv2.imshow('Mask', mask)
        cv2.imshow('Result', result)
        
        # Manejar teclas
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            self.get_logger().info("Saliendo...")
            rclpy.shutdown()
        elif key == ord('s'):
            self.save_values()
        elif key == ord('r'):
            self.reset_values()
        elif key == ord('p'):
            self.print_values()

    # Callbacks para los sliders
    def update_h_min(self, val):
        self.h_min = val
    
    def update_s_min(self, val):
        self.s_min = val
        
    def update_v_min(self, val):
        self.v_min = val
        
    def update_h_max(self, val):
        self.h_max = val
        
    def update_s_max(self, val):
        self.s_max = val
        
    def update_v_max(self, val):
        self.v_max = val
    
    def load_preset(self, val):
        """Carga valores predefinidos para colores comunes"""
        if val == 1:  # Verde
            self.h_min, self.s_min, self.v_min = 37, 143, 0
            self.h_max, self.s_max, self.v_max = 68, 255, 181
            self.get_logger().info("Preset cargado: Verde")
        elif val == 2:  # Rojo (rango inferior)
            self.h_min, self.s_min, self.v_min = 0, 50, 50
            self.h_max, self.s_max, self.v_max = 10, 255, 255
            self.get_logger().info("Preset cargado: Rojo inferior (0-10)")
        elif val == 3:  # Rojo (rango superior)
            self.h_min, self.s_min, self.v_min = 170, 50, 50
            self.h_max, self.s_max, self.v_max = 180, 255, 255
            self.get_logger().info("Preset cargado: Rojo superior (170-180)")
        elif val == 4:  # Azul
            self.h_min, self.s_min, self.v_min = 100, 50, 50
            self.h_max, self.s_max, self.v_max = 130, 255, 255
            self.get_logger().info("Preset cargado: Azul")
        else:  # Manual (no cambiar valores)
            return
            
        # Actualizar sliders
        cv2.setTrackbarPos('H Min', 'HSV Controls', self.h_min)
        cv2.setTrackbarPos('S Min', 'HSV Controls', self.s_min)
        cv2.setTrackbarPos('V Min', 'HSV Controls', self.v_min)
        cv2.setTrackbarPos('H Max', 'HSV Controls', self.h_max)
        cv2.setTrackbarPos('S Max', 'HSV Controls', self.s_max)
        cv2.setTrackbarPos('V Max', 'HSV Controls', self.v_max)
    
    def print_values(self):
        """Imprime los valores actuales en formato para copiar/pegar"""
        self.get_logger().info("\n=== VALORES ACTUALES ===")
        self.get_logger().info(f"Rango HSV: H({self.h_min}-{self.h_max}), S({self.s_min}-{self.s_max}), V({self.v_min}-{self.v_max})")
        self.get_logger().info("\nPara usar en tu código:")
        self.get_logger().info(f"lower_bound = np.array([{self.h_min}, {self.s_min}, {self.v_min}])")
        self.get_logger().info(f"upper_bound = np.array([{self.h_max}, {self.s_max}, {self.v_max}])")
        self.get_logger().info("========================\n")
        
        # También imprimir en terminal estándar para fácil copia
        print("\n=== VALORES ACTUALES ===")
        print(f"Rango HSV: H({self.h_min}-{self.h_max}), S({self.s_min}-{self.s_max}), V({self.v_min}-{self.v_max})")
        print("\nPara usar en tu código:")
        print(f"lower_bound = np.array([{self.h_min}, {self.s_min}, {self.v_min}])")
        print(f"upper_bound = np.array([{self.h_max}, {self.s_max}, {self.v_max}])")
        print("========================\n")
    
    def save_values(self):
        """Guarda los valores en un archivo"""
        filename = "hsv_calibration.txt"
        try:
            with open(filename, 'a') as f:
                f.write(f"\n# Calibración HSV ROS - {rclpy.clock.Clock().now().nanoseconds}\n")
                f.write(f"lower_bound = np.array([{self.h_min}, {self.s_min}, {self.v_min}])\n")
                f.write(f"upper_bound = np.array([{self.h_max}, {self.s_max}, {self.v_max}])\n")
            self.get_logger().info(f"Valores guardados en {filename}")
        except Exception as e:
            self.get_logger().error(f"Error guardando archivo: {e}")
    
    def reset_values(self):
        """Resetea a valores por defecto"""
        self.h_min, self.s_min, self.v_min = 0, 50, 50
        self.h_max, self.s_max, self.v_max = 180, 255, 255
        
        cv2.setTrackbarPos('H Min', 'HSV Controls', self.h_min)
        cv2.setTrackbarPos('S Min', 'HSV Controls', self.s_min)
        cv2.setTrackbarPos('V Min', 'HSV Controls', self.v_min)
        cv2.setTrackbarPos('H Max', 'HSV Controls', self.h_max)
        cv2.setTrackbarPos('S Max', 'HSV Controls', self.s_max)
        cv2.setTrackbarPos('V Max', 'HSV Controls', self.v_max)
        cv2.setTrackbarPos('Preset', 'HSV Controls', 0)
        self.get_logger().info("Valores reseteados")

def main(args=None):
    rclpy.init(args=args)
    
    try:
        calibrator = HSVCalibrator()
        rclpy.spin(calibrator)
    except KeyboardInterrupt:
        print("\nInterrumpido por el usuario")
    finally:
        cv2.destroyAllWindows()
        rclpy.shutdown()

if __name__ == "__main__":
    main()