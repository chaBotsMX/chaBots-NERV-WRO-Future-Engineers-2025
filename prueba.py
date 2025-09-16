#!/usr/bin/env python3
import qwiic_otos
import sys
import time
import math
import json
import os
from datetime import datetime

BIAS_FILE = "otos_bias.json"

def load_bias():
    if os.path.exists(BIAS_FILE):
        try:
            with open(BIAS_FILE, "r") as f:
                d = json.load(f)
            return float(d.get("dx_bias", 0.0)), float(d.get("dy_bias", 0.0)), float(d.get("dpsi_bias_deg", 0.0))
        except Exception:
            pass
    return 0.0, 0.0, 0.0

def save_bias(dx_bias, dy_bias, dpsi_bias_deg):
    try:
        with open(BIAS_FILE, "w") as f:
            json.dump({
                "dx_bias": dx_bias,
                "dy_bias": dy_bias,
                "dpsi_bias_deg": dpsi_bias_deg,
                "saved_at": datetime.utcnow().isoformat() + "Z"
            }, f, indent=2)
    except Exception:
        pass

def run():
    print("\nQwiic OTOS - Lecturas con bias por software (sin calibrateImu)\n")

    otos = qwiic_otos.QwiicOTOS()

    # Conexión
    if not otos.is_connected():
        print("El OTOS no está conectado. Revisa I2C/cableado.", file=sys.stderr)
        return
    if not otos.begin():
        print("Fallo en begin().", file=sys.stderr)
        return

    # Unidades internas: METROS y GRADOS (imprimimos en cm más abajo)
    otos.setLinearUnit(otos.kLinearUnitMeters)
    otos.setAngularUnit(otos.kAngularUnitDegrees)

    # Reset de tracking para iniciar en (0,0,0)
    otos.resetTracking()
    time.sleep(0.1)

    # Carga bias previo (si existe)
    dx_bias, dy_bias, dpsi_bias_deg = load_bias()
    print(f"Bias cargado: dx={dx_bias:.6f} m, dy={dy_bias:.6f} m, dpsi={dpsi_bias_deg:.6f} deg")

    # Parámetros
    alpha_head = 0.15        # EMA del heading (0..1)
    zupt_thresh_m = 0.001    # “quieto” si ds < 1 mm por ciclo
    zupt_thresh_deg = 0.2    # y |dpsi| < 0.2°
    bias_alpha = 0.05        # EMA para actualizar bias cuando hay ZUPT
    init_stationary_s = 3.0  # segundos de reposo al inicio para estimar bias inicial
    dt_print = 0.05          # ~20 Hz

    # Estado para deltas
    prev_x_m = None
    prev_y_m = None
    prev_h_deg_raw = None

    # “Pose soft” integrada con deltas corregidos (no la del dispositivo)
    x_soft_m = 0.0
    y_soft_m = 0.0
    h_soft_deg = 0.0

    # Heading suavizado (EMA circular)
    u_x = None
    u_y = None

    # --- Fase 1: estimación rápida de bias inicial (robot quieto) ---
    if init_stationary_s > 0:
        print(f"Mantén el robot QUIETO {init_stationary_s:.0f} s para estimar bias…")
        t0 = time.time()
        samples = 0
        sum_dx = 0.0
        sum_dy = 0.0
        sum_dpsi = 0.0

        # Leemos en lazo y promediamos deltas “crudos” (por diferencia de posición/heading)
        while time.time() - t0 < init_stationary_s:
            pos = otos.getPosition()
            x_m, y_m, h_deg = pos.x, pos.y, pos.h

            if prev_x_m is not None:
                dx_m = x_m - prev_x_m
                dy_m = y_m - prev_y_m
                dpsi_deg = h_deg - prev_h_deg_raw
                # envuelve -180..180
                dpsi_deg = (dpsi_deg + 180.0) % 360.0 - 180.0

                sum_dx += dx_m
                sum_dy += dy_m
                sum_dpsi += dpsi_deg
                samples += 1

            prev_x_m, prev_y_m, prev_h_deg_raw = x_m, y_m, h_deg
            time.sleep(dt_print)

        if samples > 0:
            dx_bias = sum_dx / samples
            dy_bias = sum_dy / samples
            dpsi_bias_deg = sum_dpsi / samples
            print(f"Bias inicial estimado: dx={dx_bias:.6f} m, dy={dy_bias:.6f} m, dpsi={dpsi_bias_deg:.6f} deg")
        # resetea previos para el bucle principal
        prev_x_m = None
        prev_y_m = None
        prev_h_deg_raw = None

    print("Entrando al bucle. Ctrl+C para salir. (Bias se guardará en archivo)")

    try:
        while True:
            pos = otos.getPosition()  # x,y en m; h en deg
            x_m, y_m, h_deg = pos.x, pos.y, pos.h

            # Deltas “crudos”
            if prev_x_m is None:
                dx_m = dy_m = dpsi_deg = 0.0
            else:
                dx_m = x_m - prev_x_m
                dy_m = y_m - prev_y_m
                dpsi_deg = h_deg - prev_h_deg_raw
                dpsi_deg = (dpsi_deg + 180.0) % 360.0 - 180.0

            prev_x_m, prev_y_m, prev_h_deg_raw = x_m, y_m, h_deg

            ds_m = math.hypot(dx_m, dy_m)

            # --- ZUPT: si estamos “quietos”, refina bias con EMA ---
            if ds_m < zupt_thresh_m and abs(dpsi_deg) < zupt_thresh_deg:
                dx_bias = (1.0 - bias_alpha) * dx_bias + bias_alpha * dx_m
                dy_bias = (1.0 - bias_alpha) * dy_bias + bias_alpha * dy_m
                dpsi_bias_deg = (1.0 - bias_alpha) * dpsi_bias_deg + bias_alpha * dpsi_deg

            # Aplica bias por software a los deltas
            dx_c = dx_m - dx_bias
            dy_c = dy_m - dy_bias
            dpsi_c_deg = dpsi_deg - dpsi_bias_deg

            # Integra pose “soft” (continua)
            x_soft_m += dx_c
            y_soft_m += dy_c
            h_soft_deg = (h_soft_deg + dpsi_c_deg + 180.0) % 360.0 - 180.0

            # Heading suavizado (EMA circular) usando h_soft
            c = math.cos(math.radians(h_soft_deg))
            s = math.sin(math.radians(h_soft_deg))
            if u_x is None:
                u_x, u_y = c, s
            else:
                u_x = (1 - alpha_head) * u_x + alpha_head * c
                u_y = (1 - alpha_head) * u_y + alpha_head * s
                n = math.hypot(u_x, u_y)
                if n > 1e-12:
                    u_x /= n
                    u_y /= n
            h_smooth_deg = math.degrees(math.atan2(u_y, u_x))

            # Impresión (m→cm)
            print()
            print("Position (soft, bias-corrected):")
            print(f"X (cm): {(x_soft_m * 100.0):.2f}")
            print(f"Y (cm): {(y_soft_m * 100.0):.2f}")
            print(f"Heading (deg) soft: {h_soft_deg:.2f}   |   smooth: {h_smooth_deg:.2f}")
            print(f"dX_c (cm): {(dx_c * 100.0):.2f}   dY_c (cm): {(dy_c * 100.0):.2f}   dS (cm): {(ds_m * 100.0):.2f}")
            print(f"Bias: dx={dx_bias:.5f} m  dy={dy_bias:.5f} m  dpsi={dpsi_bias_deg:.4f} deg")

            time.sleep(dt_print)

    except (KeyboardInterrupt, SystemExit):
        print("\nGuardando bias en archivo…")
        save_bias(dx_bias, dy_bias, dpsi_bias_deg)
        print("Fin.")
        sys.exit(0)

if __name__ == '__main__':
    run()
