#!/usr/bin/env python3
import cv2
import numpy as np
import time
from datetime import datetime
from picamera2 import Picamera2

PICAMERA2_AVAILABLE = True

def deteccion_tiempo_real_picamera2():
    try:
        picam2 = Picamera2()
        
        config = picam2.create_preview_configuration(
            main={"size": (640, 480)}
        )
        
        picam2.configure(config)
        picam2.start()
        time.sleep(2)
                
        colors = {
            'azul': {
                'lower': np.array([100, 80, 80]),
                'upper': np.array([130, 255, 255]),
                'bgr': (255, 0, 0)
            },
            'rojo': {
                'lower1': np.array([0, 80, 80]),
                'upper1': np.array([10, 255, 255]),
                'lower2': np.array([170, 80, 80]),
                'upper2': np.array([180, 255, 255]),
                'bgr': (0, 0, 255)
            }
        }
        
        frame_count = 0
        start_time = time.time()
        
        while True:
            frame_rgb = picam2.capture_array()
            
            frame = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            
            detections = 0
            
            for color_name, color_info in colors.items():
                if color_name == 'rojo':
                    mask1 = cv2.inRange(hsv, color_info['lower1'], color_info['upper1'])
                    mask2 = cv2.inRange(hsv, color_info['lower2'], color_info['upper2'])
                    mask = cv2.bitwise_or(mask1, mask2)
                else:
                    mask = cv2.inRange(hsv, color_info['lower'], color_info['upper'])

                kernel = np.ones((5, 5), np.uint8)
                mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
                mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
                
                # Encontrar contornos
                contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                
                for contour in contours:
                    area = cv2.contourArea(contour)
                    if area > 800:
                        detections += 1
                        x, y, w, h = cv2.boundingRect(contour)
                        
                        # Dibujar rectangulo
                        cv2.rectangle(frame, (x, y), (x+w, y+h), color_info['bgr'], 2)
                        cv2.putText(frame, f"{color_name.upper()} ({int(area)})", 
                                   (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color_info['bgr'], 2)
                        
                        # Dibujar punto central
                        center_x, center_y = x + w//2, y + h//2
                        cv2.circle(frame, (center_x, center_y), 5, color_info['bgr'], -1)
            
            # Calcular FPS
            frame_count += 1
            elapsed = time.time() - start_time
            if elapsed >= 1.0:
                fps = frame_count / elapsed
                frame_count = 0
                start_time = time.time()
                
                # Mostrar info en consola
                print(f"? FPS: {fps:.1f} | Detecciones: {detections}")
              
            # Mostrar frame en ventana
            cv2.imshow('Pi Camera - Video en Vivo', frame)
            
            # Procesar teclas
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                print("? Detecci�n terminada")
                break
            elif key == ord('s'):
                timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                filename = f"picamera_capture_{timestamp}.jpg"
                cv2.imwrite(filename, frame)
                print(f"? Imagen guardada: {filename}")
        
        picam2.stop()
        cv2.destroyAllWindows()
        return True
        
    except Exception as e:
        print(f"Error tiempo real: {e}")
        return False

def main():
    deteccion_tiempo_real_picamera2()
    
if __name__ == "__main__":
    main()