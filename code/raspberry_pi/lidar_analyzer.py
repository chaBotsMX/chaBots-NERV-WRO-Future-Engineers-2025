import subprocess
import re
import math
from collections import defaultdict


class LidarSectorAnalyzer:
    def __init__(self):
        self.target_angles = [0, 90, 180, 270]
        self.angle_tolerance = 5

        self.sector_data = {angle: [] for angle in self.target_angles}

        self.data_pattern = re.compile(r'theta:\s*(\d+\.\d+)\s+Dist:\s*(\d+\.\d+)\s+Q:\s*(\d+)', re.IGNORECASE)

        self.sample_count = 0
        self.report_interval = 20

    def normalize_angle(self, angle):
        """Normaliza el �ngulo al rango [0, 360)"""
        return angle % 360

    def is_in_sector(self, angle, target_angle):
        angle = self.normalize_angle(angle)
        target = self.normalize_angle(target_angle)

        if target == 0:
            return (angle <= self.angle_tolerance) or (angle >= (360 - self.angle_tolerance))
        else:
            return abs(angle - target) <= self.angle_tolerance

    def process_reading(self, angle, distance, quality):
        if quality < 20 or distance == 0:
            return

        for target_angle in self.target_angles:
            if self.is_in_sector(angle, target_angle):
                self.sector_data[target_angle].append({
                    'angle': angle,
                    'distance': distance,
                    'quality': quality
                })
                break

        self.sample_count += 1

        if self.sample_count % self.report_interval == 0:
            self.print_current_averages()

    def calculate_sector_average(self, target_angle):
        """Calcula el promedio de distancias para un sector espec�fico"""
        data = self.sector_data[target_angle]
        if not data:
            return None, 0

        distances = [reading['distance'] for reading in data]
        average = sum(distances) / len(distances)
        return average, len(distances)

    def print_current_averages(self):
        """Imprime los promedios actuales de todos los sectores"""
        print(f"\n{'='*60}")
        print(f"ESTAD�STICAS DESPU�S DE {self.sample_count} MUESTRAS")
        print(f"{'='*60}")

        for target_angle in self.target_angles:
            avg_dist, count = self.calculate_sector_average(target_angle)
            if avg_dist is not None:
                print(f"Sector {target_angle:3d}� (�{self.angle_tolerance}�): "
                      f"Promedio = {avg_dist:6.1f} mm, Muestras = {count:3d}")
            else:
                print(f"Sector {target_angle:3d}� (�{self.angle_tolerance}�): "
                      f"Sin datos")
        print(f"{'='*60}\n")

def process_lidar_stream():
    analyzer = LidarSectorAnalyzer()

    try:
        proc = subprocess.Popen(
            ["./ultra_simple", "--channel", "--serial", "/dev/ttyUSB0", "460800"],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            bufsize=1
        )

        while True:
            line = proc.stdout.readline()
            if line:
                match = analyzer.data_pattern.search(line)
                if match:
                    angle = float(match.group(1))
                    dist = float(match.group(2))
                    quality = int(match.group(3))

                    analyzer.process_reading(angle, dist, quality)

            else:
                err = proc.stderr.readline()
                if err:
                    print(f"STDERR: {err.strip()}")

    except FileNotFoundError:
        print("ERROR: No se encontr� el ejecutable './ultra_simple'")
        print("Aseg�rate de que el archivo existe y tiene permisos de ejecuci�n")
    except Exception as e:
        print(f"ERROR al iniciar ultra_simple: {e}")
    except KeyboardInterrupt:
        print("\nTerminando el proceso...")
        if 'proc' in locals():
            proc.terminate()
            proc.wait()

if __name__ == "__main__":
    process_lidar_stream()
