#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os, math
import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String, Float32
from geometry_msgs.msg import Point
from cv_bridge import CvBridge

class ColorDetectionNode(Node):
    # Entradas/salidas
    TOPIC_IN   = '/camera/image_raw'
    TOPIC_MASK = '/color_detection/mask'          # mono8
    TOPIC_ANN  = '/color_detection/annotated'     # mono8
    TOPIC_CROP = '/color_detection/cropped'       # bgr8

    # Crop (0..1 del alto)
    CROP_TOP_PCT    = 0.00
    CROP_BOTTOM_PCT = 0.00

    # Óptica (aprox) y umbrales
    FOV_H_DEG = 60.0
    FOCAL_PX = 500.0
    OBJECT_REAL_SIZE_M = 0.05
    MIN_AREA_PX = 500
    GREEN_LOWER = np.array([40, 150, 100], dtype=np.uint8)
    GREEN_UPPER = np.array([75, 255, 255], dtype=np.uint8)
    RED_LOWER1  = np.array([  0, 150, 150], dtype=np.uint8)
    RED_UPPER1  = np.array([ 15, 255, 255], dtype=np.uint8)
    RED_LOWER2  = np.array([170, 150, 150], dtype=np.uint8)
    RED_UPPER2  = np.array([179, 255, 255], dtype=np.uint8)

    def __init__(self):
        super().__init__('color_detection_node')
        self.bridge = CvBridge()

        # Publishers (aparecen en ros2 topic list aunque no haya datos)
        self.pub_mask = self.create_publisher(Image, self.TOPIC_MASK, 10)
        self.pub_ann  = self.create_publisher(Image, self.TOPIC_ANN, 10)
        self.pub_crop = self.create_publisher(Image, self.TOPIC_CROP, 10)
        self.pub_color    = self.create_publisher(String,  '/detected_color', 10)
        self.pub_distance = self.create_publisher(Float32, '/object_distance', 10)
        self.pub_angle    = self.create_publisher(Float32, '/object_angle', 10)
        self.pub_position = self.create_publisher(Point,   '/object_position', 10)

        # Subscriber
        self.sub = self.create_subscription(Image, self.TOPIC_IN, self.cb, 10)

        self.img_w = 640
        self.img_h = 480
        self.x_off = 0
        self.y_off = 0

        self.get_logger().info(f'Running: {os.path.realpath(__file__)}')
        self.get_logger().info(f'Sub: {self.TOPIC_IN} | Pub: {self.TOPIC_MASK}, {self.TOPIC_ANN}, {self.TOPIC_CROP}')

    def cb(self, msg: Image):
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

        # Recorte vertical
        h, w = frame.shape[:2]
        top = max(0, min(h - 1, int(h * self.CROP_TOP_PCT)))
        bot = max(top + 1, min(h, int(h * self.CROP_BOTTOM_PCT)))
        roi = frame[top:bot, :].copy()

        # Publicar recorte BGR
        crop_msg = self.bridge.cv2_to_imgmsg(roi, 'bgr8'); crop_msg.header = msg.header
        self.pub_crop.publish(crop_msg)

        # Guardar offsets y tamaño
        self.x_off = 0; self.y_off = top
        self.img_h, self.img_w = roi.shape[:2]

        # HSV y máscaras
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        g = cv2.inRange(hsv, self.GREEN_LOWER, self.GREEN_UPPER)
        r = cv2.bitwise_or(
            cv2.inRange(hsv, self.RED_LOWER1, self.RED_UPPER1),
            cv2.inRange(hsv, self.RED_LOWER2, self.RED_UPPER2)
        )

        # Limpieza
        k = np.ones((5,5), np.uint8)
        g = cv2.morphologyEx(cv2.morphologyEx(g, cv2.MORPH_OPEN, k), cv2.MORPH_CLOSE, k)
        r = cv2.morphologyEx(cv2.morphologyEx(r, cv2.MORPH_OPEN, k), cv2.MORPH_CLOSE, k)

        # Publicar máscara (mono8)
        mask = cv2.bitwise_or(g, r)
        mask_msg = self.bridge.cv2_to_imgmsg(mask, 'mono8'); mask_msg.header = msg.header
        self.pub_mask.publish(mask_msg)

        # Buscar mejor contorno
        best = None
        for name, m in (('green', g), ('red', r)):
            cnts, _ = cv2.findContours(m, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if not cnts: continue
            c = max(cnts, key=cv2.contourArea)
            area = float(cv2.contourArea(c))
            if area < self.MIN_AREA_PX: continue
            M = cv2.moments(c); 
            if M['m00'] == 0: continue
            cx, cy = int(M['m10']/M['m00']), int(M['m01']/M['m00'])
            if best is None or area > best[3]: best = (name, cx, cy, area, c)

        # Anotación (mono8)
        ann = np.zeros((roi.shape[0], roi.shape[1]), dtype=np.uint8)
        ann[mask > 0] = 255
        if best:
            name, cx, cy, area, c = best
            cv2.circle(ann, (cx, cy), 8, 255, -1)
            cv2.drawContours(ann, [c], -1, 255, 2)
            dist = self._dist(area); ang = self._ang(cx)
            cv2.putText(ann, f'{name} d={dist:.2f}m a={ang:.1f}deg',
                        (max(0, cx-80), max(15, cy-15)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, 255, 1, cv2.LINE_AA)

            # Publicar mediciones
            pos = Point(x=float(cx + self.x_off), y=float(cy + self.y_off), z=float(dist))
            self._pub_meas(name, dist, ang, pos)
        else:
            self._pub_meas('none', 0.0, 0.0, Point())

        ann_msg = self.bridge.cv2_to_imgmsg(ann, 'mono8'); ann_msg.header = msg.header
        self.pub_ann.publish(ann_msg)

    def _dist(self, area: float) -> float:
        if area <= 0: return 0.0
        d = (self.OBJECT_REAL_SIZE_M * self.FOCAL_PX) / math.sqrt(area)
        return float(max(0.1, min(d, 10.0)))

    def _ang(self, x: int) -> float:
        cx = self.img_w / 2.0
        return float((x - cx)/cx * (self.FOV_H_DEG/2.0))

    def _pub_meas(self, color, dist, ang, pos):
        self.pub_color.publish(String(data=color))
        self.pub_distance.publish(Float32(data=dist))
        self.pub_angle.publish(Float32(data=ang))
        self.pub_position.publish(pos)

def main(args=None):
    rclpy.init(args=args)
    node = ColorDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
