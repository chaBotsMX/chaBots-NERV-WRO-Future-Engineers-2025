#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import time
import json
import os
import sys
import traceback
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from builtin_interfaces.msg import Time as TimeMsg

try:
    from tf2_ros import TransformBroadcaster
except Exception:
    TransformBroadcaster = None  # por si no está tf2_ros

# Librería SparkFun
import qwiic_otos


def yaw_to_quat(z_rad: float):
    """Convierte yaw (rad) a quaternion (x,y,z,w). Asume roll=pitch=0."""
    half = 0.5 * z_rad
    cz = math.cos(half)
    sz = math.sin(half)
    # (x,y,z,w) para yaw puro:
    return (0.0, 0.0, sz, cz)

def wrap_deg(a):
    return (a + 180.0) % 360.0 - 180.0

def now_msg(node: Node) -> TimeMsg:
    t = node.get_clock().now().to_msg()
    return t


class OtosRosNode(Node):
    def __init__(self):
        super().__init__('otos_node')

        # ----------------------------
        # Parámetros ROS 2
        # ----------------------------
        self.declare_parameter('rate_hz', 30.0)
        self.declare_parameter('frame_id', 'odom')
        self.declare_parameter('child_frame_id', 'base_link')
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('bias_file', 'otos_bias.json')

        # ZUPT / smoothing
        self.declare_parameter('zupt_thresh_m', 0.001)       # <1 mm por ciclo => quieto
        self.declare_parameter('zupt_thresh_deg', 0.2)
        self.declare_parameter('heading_ema_alpha', 0.15)    # 0..1
        self.declare_parameter('bias_ema_alpha', 0.05)       # 0..1

        # Reconnect
        self.declare_parameter('auto_reconnect', True)
        self.declare_parameter('reconnect_backoff_initial', 0.5)
        self.declare_parameter('reconnect_backoff_max', 4.0)

        # Dispositivo / unidades / escalas
        self.declare_parameter('linear_unit_meters', True)    # True => m, False => inches
        self.declare_parameter('angular_unit_degrees', True)  # True => deg, False => rad
        self.declare_parameter('linear_scalar', 1.0)          # opcional (None = no tocar)
        self.declare_parameter('angular_scalar', 1.0)         # opcional
        self.declare_parameter('signal_process_config', -1)   # -1 = default firmware

        # Reset tracking al (re)conectar
        self.declare_parameter('reset_tracking_on_connect', True)

        # Lee parámetros
        self.rate_hz = float(self.get_parameter('rate_hz').value)
        self.frame_id = str(self.get_parameter('frame_id').value)
        self.child_frame_id = str(self.get_parameter('child_frame_id').value)
        self.publish_tf = bool(self.get_parameter('publish_tf').value)
        self.bias_file = str(self.get_parameter('bias_file').value)

        self.zupt_thresh_m = float(self.get_parameter('zupt_thresh_m').value)
        self.zupt_thresh_deg = float(self.get_parameter('zupt_thresh_deg').value)
        self.heading_ema_alpha = float(self.get_parameter('heading_ema_alpha').value)
        self.bias_ema_alpha = float(self.get_parameter('bias_ema_alpha').value)

        self.auto_reconnect = bool(self.get_parameter('auto_reconnect').value)
        self.reconnect_backoff_initial = float(self.get_parameter('reconnect_backoff_initial').value)
        self.reconnect_backoff_max = float(self.get_parameter('reconnect_backoff_max').value)

        self.linear_unit_meters = bool(self.get_parameter('linear_unit_meters').value)
        self.angular_unit_degrees = bool(self.get_parameter('angular_unit_degrees').value)
        self.linear_scalar = float(self.get_parameter('linear_scalar').value)
        self.angular_scalar = float(self.get_parameter('angular_scalar').value)
        spc = int(self.get_parameter('signal_process_config').value)
        self.signal_process_config = None if spc < 0 else int(spc)

        self.reset_tracking_on_connect = bool(self.get_parameter('reset_tracking_on_connect').value)

        # ----------------------------
        # Publicadores
        # ----------------------------
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.pub_odom = self.create_publisher(Odometry, 'odom', qos)

        self.tf_broadcaster = None
        if self.publish_tf and TransformBroadcaster is not None:
            self.tf_broadcaster = TransformBroadcaster(self)

        # ----------------------------
        # Estado interno
        # ----------------------------
        self.dev: Optional[qwiic_otos.QwiicOTOS] = None

        # Bias por software (persistente)
        self.dx_bias = 0.0
        self.dy_bias = 0.0
        self.dpsi_bias_deg = 0.0
        self._load_bias_file()

        # Previos de lectura cruda (del dispositivo)
        self.prev_x_m = None
        self.prev_y_m = None
        self.prev_h_deg_raw = None

        # Pose “soft” (integración de deltas corregidos)
        self.x_soft_m = 0.0
        self.y_soft_m = 0.0
        self.h_soft_deg = 0.0

        # Smoothing circular del heading
        self.u_x = None
        self.u_y = None

        # Timer principal
        period = 1.0 / max(1e-6, self.rate_hz)
        self.timer = self.create_timer(period, self._on_timer)

        # Conectar dispositivo al inicio
        if not self._connect_and_config():
            self.get_logger().warn("No se pudo conectar al OTOS. Intentando reconectar en background...")
            # dejamos que el _on_timer haga el reconnect loop cuando falle la lectura

        self.get_logger().info("OTOS ROS2 node listo.")

    # ------------------------------------------------------------------
    # Conexión y configuración
    # ------------------------------------------------------------------
    def _connect_and_config(self) -> bool:
        try:
            self.dev = qwiic_otos.QwiicOTOS()
            if not self.dev.is_connected():
                self.get_logger().error("OTOS no detectado en I2C.")
                self.dev = None
                return False

            if not self.dev.begin():
                self.get_logger().error("begin() falló.")
                self.dev = None
                return False

            # Unidades internas
            if self.linear_unit_meters:
                self.dev.setLinearUnit(self.dev.kLinearUnitMeters)
            else:
                self.dev.setLinearUnit(self.dev.kLinearUnitInches)
            if self.angular_unit_degrees:
                self.dev.setAngularUnit(self.dev.kAngularUnitDegrees)
            else:
                self.dev.setAngularUnit(self.dev.kAngularUnitRadians)

            # Escalas (si difieren de 1.0)
            if self.linear_scalar and abs(self.linear_scalar - 1.0) > 1e-6:
                self.dev.setLinearScalar(float(self.linear_scalar))
            if self.angular_scalar and abs(self.angular_scalar - 1.0) > 1e-6:
                self.dev.setAngularScalar(float(self.angular_scalar))

            # Flags de procesamiento de señal, si se han definido
            if self.signal_process_config is not None:
                self.dev.setSignalProcessConfig(int(self.signal_process_config))

            if self.reset_tracking_on_connect:
                self._reset_tracking_filters_only()

            self.get_logger().info("OTOS conectado y configurado.")
            return True

        except Exception as e:
            self.get_logger().error(f"Excepción conectando OTOS: {e}")
            self.get_logger().debug(traceback.format_exc())
            self.dev = None
            return False

    def _reset_tracking_filters_only(self):
        try:
            self.dev.resetTracking()
            time.sleep(0.05)
        except Exception:
            pass
        # Resetea estados temporales de integración/suavizado
        self.prev_x_m = None
        self.prev_y_m = None
        self.prev_h_deg_raw = None
        self.u_x = None
        self.u_y = None
        # NO reseteamos x_soft/y_soft/h_soft: queremos continuidad externa

    def _reconnect_loop(self):
        if not bool(self.auto_reconnect):
            self.get_logger().error("Auto-reconnect deshabilitado. Deteniendo nodo.")
            rclpy.shutdown()
            return

        backoff = float(self.reconnect_backoff_initial)
        while rclpy.ok():
            self.get_logger().warn(f"Intentando reconectar OTOS en {backoff:.2f}s...")
            time.sleep(backoff)
            if self._connect_and_config():
                self.get_logger().info("Reconectado OTOS.")
                return
            backoff = min(float(self.reconnect_backoff_max), backoff * 2.0)

    # ------------------------------------------------------------------
    # Bias persistente
    # ------------------------------------------------------------------
    def _load_bias_file(self):
        try:
            if os.path.exists(self.bias_file):
                with open(self.bias_file, 'r') as f:
                    d = json.load(f)
                self.dx_bias = float(d.get('dx_bias', 0.0))
                self.dy_bias = float(d.get('dy_bias', 0.0))
                self.dpsi_bias_deg = float(d.get('dpsi_bias_deg', 0.0))
                self.get_logger().info(f"Bias cargado: dx={self.dx_bias:.6f} m, dy={self.dy_bias:.6f} m, dpsi={self.dpsi_bias_deg:.6f} deg")
        except Exception as e:
            self.get_logger().warn(f"No se pudo cargar bias_file '{self.bias_file}': {e}")

    def _save_bias_file(self):
        try:
            data = {
                'dx_bias': self.dx_bias,
                'dy_bias': self.dy_bias,
                'dpsi_bias_deg': self.dpsi_bias_deg,
                'saved_at': self.get_clock().now().to_msg().sec
            }
            with open(self.bias_file, 'w') as f:
                json.dump(data, f, indent=2)
            self.get_logger().info(f"Bias guardado en {self.bias_file}")
        except Exception as e:
            self.get_logger().warn(f"No se pudo guardar bias_file: {e}")

    # ------------------------------------------------------------------
    # Lazo principal
    # ------------------------------------------------------------------
    def _safe_get_position(self):
        if self.dev is None:
            raise RuntimeError("OTOS no inicializado")

        try:
            pos = self.dev.getPosition()
            if pos is None:
                raise RuntimeError("getPosition() devolvió None.")
            return pos
        except Exception as e:
            self.get_logger().error(f"Error de lectura OTOS: {e}")
            self.dev = None
            self._reconnect_loop()
            if self.dev is None:
                raise RuntimeError("Reconexión fallida.")
            pos = self.dev.getPosition()
            if pos is None:
                raise RuntimeError("getPosition() devolvió None tras reconexión.")
            return pos

    def _on_timer(self):
        # 1) Leer posición cruda del dispositivo (en m / deg por config)
        try:
            pos = self._safe_get_position()
        except Exception as e:
            self.get_logger().error(f"No se pudo obtener posición: {e}")
            return

        x_m = float(pos.x)
        y_m = float(pos.y)
        h_deg_raw = float(pos.h)

        # 2) Calcular deltas crudos entre lecturas
        if self.prev_x_m is None:
            dx_m = dy_m = dpsi_deg = 0.0
        else:
            dx_m = x_m - self.prev_x_m
            dy_m = y_m - self.prev_y_m
            dpsi_deg = wrap_deg(h_deg_raw - self.prev_h_deg_raw)

        self.prev_x_m = x_m
        self.prev_y_m = y_m
        self.prev_h_deg_raw = h_deg_raw

        ds_m = math.hypot(dx_m, dy_m)

        # 3) ZUPT → refinar bias (solo cuando “quieto”)
        if ds_m < self.zupt_thresh_m and abs(dpsi_deg) < self.zupt_thresh_deg:
            a = self.bias_ema_alpha
            self.dx_bias = (1.0 - a) * self.dx_bias + a * dx_m
            self.dy_bias = (1.0 - a) * self.dy_bias + a * dy_m
            self.dpsi_bias_deg = (1.0 - a) * self.dpsi_bias_deg + a * dpsi_deg

        # 4) Aplicar bias a los deltas y actualizar pose “soft”
        dx_c = dx_m - self.dx_bias
        dy_c = dy_m - self.dy_bias
        dpsi_c_deg = dpsi_deg - self.dpsi_bias_deg

        self.x_soft_m += dx_c
        self.y_soft_m += dy_c
        self.h_soft_deg = wrap_deg(self.h_soft_deg + dpsi_c_deg)

        # 5) EMA circular del heading “soft”
        c = math.cos(math.radians(self.h_soft_deg))
        s = math.sin(math.radians(self.h_soft_deg))
        if self.u_x is None:
            self.u_x, self.u_y = c, s
        else:
            a = self.heading_ema_alpha
            self.u_x = (1.0 - a) * self.u_x + a * c
            self.u_y = (1.0 - a) * self.u_y + a * s
            n = math.hypot(self.u_x, self.u_y)
            if n > 1e-12:
                self.u_x /= n
                self.u_y /= n

        h_smooth_deg = math.degrees(math.atan2(self.u_y, self.u_x))
        yaw_rad = math.radians(h_smooth_deg)

        # 6) Publicar Odom y TF
        self._publish_odometry(self.x_soft_m, self.y_soft_m, yaw_rad, dx_c, dy_c)
        if self.tf_broadcaster is not None:
            self._publish_tf(self.x_soft_m, self.y_soft_m, yaw_rad)

        # 7) Guarda bias de vez en cuando (opcional: aquí simple cada N ticks)
        #    Para no escribir demasiado, podrías hacerlo cada X segundos.
        #    Aquí lo omitimos; puedes habilitarlo si lo necesitas:
        # if self.get_clock().now().nanoseconds % int(5e9) < 1e7:
        #     self._save_bias_file()

    # ------------------------------------------------------------------
    # Publicación
    # ------------------------------------------------------------------
    def _publish_odometry(self, x, y, yaw, dx_c, dy_c):
        msg = Odometry()
        msg.header.stamp = now_msg(self)
        msg.header.frame_id = self.frame_id
        msg.child_frame_id = self.child_frame_id

        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.position.z = 0.0

        qx, qy, qz, qw = yaw_to_quat(yaw)
        msg.pose.pose.orientation.x = qx
        msg.pose.pose.orientation.y = qy
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw

        # Velocidad (aprox por delta * rate)
        # Nota: dx_c, dy_c están en el marco del dispositivo (odom); si quisieras vel en base_link, rota.
        vx = dx_c * self.rate_hz
        vy = dy_c * self.rate_hz
        msg.twist.twist.linear.x = vx
        msg.twist.twist.linear.y = vy
        msg.twist.twist.linear.z = 0.0
        # angular z ~ derivada de yaw suavizado
        # (opcional, aquí omitimos suavizado de derivada por simplicidad)
        # msg.twist.twist.angular.z = ...

        # Covarianzas básicas (ajusta según tus mediciones)
        # Pose
        msg.pose.covariance = [
            1e-3, 0,    0, 0, 0, 0,
            0,    1e-3, 0, 0, 0, 0,
            0,    0,    1e6,0, 0, 0,
            0,    0,    0, 1e6,0, 0,
            0,    0,    0, 0, 1e6,0,
            0,    0,    0, 0, 0, 1e-2
        ]
        # Twist
        msg.twist.covariance = [
            1e-2, 0,    0, 0, 0, 0,
            0,    1e-2, 0, 0, 0, 0,
            0,    0,    1e6,0, 0, 0,
            0,    0,    0, 1e6,0, 0,
            0,    0,    0, 0, 1e6,0,
            0,    0,    0, 0, 0, 5e-2
        ]

        self.pub_odom.publish(msg)

    def _publish_tf(self, x, y, yaw):
        t = TransformStamped()
        t.header.stamp = now_msg(self)
        t.header.frame_id = self.frame_id
        t.child_frame_id = self.child_frame_id
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = 0.0
        qx, qy, qz, qw = yaw_to_quat(yaw)
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(t)

    # ------------------------------------------------------------------
    # Ciclo de vida
    # ------------------------------------------------------------------
    def destroy_node(self):
        # Guarda bias al salir
        try:
            self._save_bias_file()
        except Exception:
            pass
        super().destroy_node()


def main():
    rclpy.init()
    node = OtosRosNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        node.get_logger().error(f"Excepción en spin: {e}")
        node.get_logger().debug(traceback.format_exc())
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
