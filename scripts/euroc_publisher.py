#!/usr/bin/env python3
"""
EuRoC MAV Dataset Publisher for ROS2.

Reads the raw EuRoC dataset (images + IMU CSV) and publishes them as ROS2 topics:
  - /cam0/image_raw  (sensor_msgs/Image)
  - /cam1/image_raw  (sensor_msgs/Image)
  - /imu0            (sensor_msgs/Imu)

Usage:
  # Default: real-time playback
  python3 euroc_publisher.py ~/dataset/Euroc/vicon_room1/V1_01_easy/mav0

  # 2x speed
  python3 euroc_publisher.py ~/dataset/Euroc/vicon_room1/V1_01_easy/mav0 --speed 2.0

  # Start from a specific offset (seconds from beginning)
  python3 euroc_publisher.py ~/dataset/Euroc/vicon_room1/V1_01_easy/mav0 --start 10.0

For use with VINS-Fusion, make sure the config.yaml has matching topic names:
  imu_topic: "/imu0"
  image0_topic: "/cam0/image_raw"
  image1_topic: "/cam1/image_raw"
"""

import os
import sys
import csv
import time
import argparse
import threading
from pathlib import Path

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from cv_bridge import CvBridge
from sensor_msgs.msg import Image, Imu
from geometry_msgs.msg import Vector3
from builtin_interfaces.msg import Time as RosTime


class EuRoCPublisher(Node):
    """Publish EuRoC dataset images and IMU data as ROS2 topics."""

    def __init__(self, dataset_path: str, speed: float = 1.0, start_offset: float = 0.0):
        super().__init__('euroc_publisher')

        self.dataset_path = Path(dataset_path)
        self.speed = speed
        self.start_offset = start_offset
        self.bridge = CvBridge()

        # ---- QoS: RELIABLE to match VINS-Fusion subscribers ----
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # ---- Publishers ----
        self.cam0_pub = self.create_publisher(Image, '/cam0/image_raw', sensor_qos)
        self.cam1_pub = self.create_publisher(Image, '/cam1/image_raw', sensor_qos)
        self.imu_pub  = self.create_publisher(Imu, '/imu0', sensor_qos)

        # ---- Load all data ----
        self.get_logger().info(f'Loading dataset from {self.dataset_path}...')

        self.cam0_data = self._load_camera_csv('cam0')
        self.cam1_data = self._load_camera_csv('cam1')
        self.imu_data  = self._load_imu_csv()

        self.get_logger().info(
            f'Loaded: cam0={len(self.cam0_data)} images, '
            f'cam1={len(self.cam1_data)} images, '
            f'imu={len(self.imu_data)} measurements'
        )

        # ---- Time range ----
        all_ts = []
        for ts, _ in self.cam0_data: all_ts.append(ts)
        for ts, _ in self.cam1_data: all_ts.append(ts)
        for ts, _, _, _, _, _, _ in self.imu_data: all_ts.append(ts)

        if not all_ts:
            self.get_logger().error('No data found! Check dataset path.')
            self.t0_ns = 0
            self.t_end_ns = 0
            self.start_ns = 0
        else:
            self.t0_ns = min(all_ts)
            self.t_end_ns = max(all_ts)
            self.start_ns = self.t0_ns + int(self.start_offset * 1e9)

        duration_s = (self.t_end_ns - self.t0_ns) / 1e9
        self.get_logger().info(
            f'Dataset duration: {duration_s:.1f}s | '
            f'Speed: {self.speed}x | Start: t+{self.start_offset}s'
        )

        # ---- Threading state ----
        self.running = True
        self.wall_start_ns = 0  # monotonic ns when playback begins

    # ═══════════════════════════════════════════════════════════════
    # CSV Loaders
    # ═══════════════════════════════════════════════════════════════

    def _load_camera_csv(self, cam_name: str) -> list:
        """Load cam0/data.csv or cam1/data.csv → [(ts_ns, filename), ...]."""
        csv_path = self.dataset_path / cam_name / 'data.csv'
        if not csv_path.exists():
            self.get_logger().warn(f'{csv_path} not found')
            return []

        data = []
        with open(csv_path, 'r') as f:
            reader = csv.reader(f)
            next(reader, None)  # skip header
            for row in reader:
                if len(row) >= 2:
                    data.append((int(row[0]), row[1].strip()))
        return data

    def _load_imu_csv(self) -> list:
        """
        Load imu0/data.csv → [(ts_ns, wx, wy, wz, ax, ay, az), ...].
        Units: gyro=rad/s, accel=m/s².
        """
        csv_path = self.dataset_path / 'imu0' / 'data.csv'
        if not csv_path.exists():
            self.get_logger().warn(f'{csv_path} not found')
            return []

        data = []
        with open(csv_path, 'r') as f:
            reader = csv.reader(f)
            next(reader, None)  # skip header
            for row in reader:
                if len(row) >= 7:
                    data.append((
                        int(row[0]),
                        float(row[1]), float(row[2]), float(row[3]),
                        float(row[4]), float(row[5]), float(row[6]),
                    ))
        return data

    # ═══════════════════════════════════════════════════════════════
    # Helpers
    # ═══════════════════════════════════════════════════════════════

    @staticmethod
    def _ns_to_ros_time(ts_ns: int) -> RosTime:
        """Nanosecond int → ROS2 Time message."""
        return RosTime(sec=int(ts_ns // 1_000_000_000),
                       nanosec=int(ts_ns % 1_000_000_000))

    def _load_image(self, cam_name: str, fname: str) -> np.ndarray:
        """Read a grayscale PNG from cam/data/."""
        img_path = self.dataset_path / cam_name / 'data' / fname
        return cv2.imread(str(img_path), cv2.IMREAD_GRAYSCALE) if img_path.exists() else None

    def _sleep_until(self, target_ns: int):
        """
        Sleep so that *wall-clock elapsed* matches *dataset elapsed* ÷ speed.
        """
        if self.wall_start_ns == 0:
            return
        data_elapsed_s = (target_ns - self.t0_ns) / 1e9
        target_wall_s = data_elapsed_s / self.speed
        current_wall_s = (time.monotonic_ns() - self.wall_start_ns) / 1e9
        delay = target_wall_s - current_wall_s
        if delay > 0:
            time.sleep(delay)

    # ═══════════════════════════════════════════════════════════════
    # Playback threads
    # ═══════════════════════════════════════════════════════════════

    def _run_cameras(self):
        """Publish cam0 + cam1 images in timestamp order."""
        self.get_logger().info('Camera thread started')

        # Merge both camera streams and sort
        events = []
        for ts, fname in self.cam0_data:
            if ts >= self.start_ns:
                events.append((ts, 'cam0', fname))
        for ts, fname in self.cam1_data:
            if ts >= self.start_ns:
                events.append((ts, 'cam1', fname))
        events.sort(key=lambda x: x[0])

        self.get_logger().info(f'Camera events: {len(events)}')

        for ts_ns, cam_name, fname in events:
            if not self.running:
                break

            self._sleep_until(ts_ns)

            img = self._load_image(cam_name, fname)
            if img is None:
                self.get_logger().warn(f'Failed to load {cam_name}/data/{fname}')
                continue

            msg = self.bridge.cv2_to_imgmsg(img, encoding='mono8')
            msg.header.stamp = self._ns_to_ros_time(ts_ns)
            msg.header.frame_id = f'{cam_name}_optical_frame'

            if cam_name == 'cam0':
                self.cam0_pub.publish(msg)
            else:
                self.cam1_pub.publish(msg)

        self.get_logger().info('Camera thread finished')

    def _run_imu(self):
        """Publish IMU data in timestamp order."""
        self.get_logger().info('IMU thread started')

        imu_subset = [d for d in self.imu_data if d[0] >= self.start_ns]
        self.get_logger().info(f'IMU events: {len(imu_subset)}')

        for ts_ns, wx, wy, wz, ax, ay, az in imu_subset:
            if not self.running:
                break

            self._sleep_until(ts_ns)

            msg = Imu()
            msg.header.stamp = self._ns_to_ros_time(ts_ns)
            msg.header.frame_id = 'imu0'
            msg.angular_velocity = Vector3(x=wx, y=wy, z=wz)
            msg.linear_acceleration = Vector3(x=ax, y=ay, z=az)

            # -1 = covariance unknown
            msg.orientation_covariance[0] = -1.0
            msg.angular_velocity_covariance[0] = -1.0
            msg.linear_acceleration_covariance[0] = -1.0

            self.imu_pub.publish(msg)

        self.get_logger().info('IMU thread finished')

    # ═══════════════════════════════════════════════════════════════
    # Entry point
    # ═══════════════════════════════════════════════════════════════

    def start(self):
        """Launch playback threads."""
        self.get_logger().info('Starting in 1s (let subscribers connect)...')
        time.sleep(1.0)
        self.wall_start_ns = time.monotonic_ns()

        t_cam = threading.Thread(target=self._run_cameras, daemon=True)
        t_imu = threading.Thread(target=self._run_imu, daemon=True)
        t_cam.start()
        t_imu.start()
        t_cam.join()
        t_imu.join()

        actual_elapsed = (time.monotonic_ns() - self.wall_start_ns) / 1e9
        self.get_logger().info(f'Playback finished in {actual_elapsed:.1f}s (wall clock)')

    def stop(self):
        self.running = False


# ═══════════════════════════════════════════════════════════════════
# CLI
# ═══════════════════════════════════════════════════════════════════

def main():
    parser = argparse.ArgumentParser(
        description='Publish EuRoC MAV dataset as ROS2 topics'
    )
    parser.add_argument(
        'dataset_path',
        help='Path to the mav0/ directory (e.g. ~/dataset/Euroc/.../mav0)',
    )
    parser.add_argument(
        '--speed', '-s', type=float, default=1.0,
        help='Playback speed multiplier (default: 1.0 = real-time)',
    )
    parser.add_argument(
        '--start', '-t', type=float, default=0.0,
        help='Start offset in seconds from the first data point (default: 0)',
    )
    args = parser.parse_args()

    rclpy.init(args=sys.argv)

    node = EuRoCPublisher(
        dataset_path=os.path.expanduser(args.dataset_path),
        speed=args.speed,
        start_offset=args.start,
    )

    try:
        playback = threading.Thread(target=node.start, daemon=True)
        playback.start()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('\nStopped by user')
        node.stop()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
