#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from collections import deque
import argparse
import math
import csv
from typing import Deque
from pathlib import Path

from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry


class TopicStats:
    def __init__(self, window_sec: float = 5.0):
        self.window_sec = window_sec
        self.arrival_times: Deque[float] = deque()
        self.header_delays: Deque[float] = deque()
        self.intervals: Deque[float] = deque()
        self.last_arrival: float | None = None

    def push(self, now_sec: float, header_sec: float):
        self.arrival_times.append(now_sec)
        self.header_delays.append(max(0.0, now_sec - header_sec))
        if self.last_arrival is not None:
            self.intervals.append(max(0.0, now_sec - self.last_arrival))
        self.last_arrival = now_sec
        while self.arrival_times and now_sec - self.arrival_times[0] > self.window_sec:
            self.arrival_times.popleft()
        while self.header_delays and len(self.header_delays) > len(self.arrival_times):
            self.header_delays.popleft()
        while self.intervals and len(self.intervals) > max(0, len(self.arrival_times) - 1):
            self.intervals.popleft()

    def rate(self) -> float:
        if not self.arrival_times:
            return 0.0
        span = self.arrival_times[-1] - self.arrival_times[0]
        if span <= 0.0:
            return float(len(self.arrival_times)) / max(1e-6, self.window_sec)
        return float(len(self.arrival_times) - 1) / span if len(self.arrival_times) > 1 else 0.0

    def delay_mean(self) -> float:
        if not self.header_delays:
            return 0.0
        return sum(self.header_delays) / len(self.header_delays)

    def jitter_std(self) -> float:
        if len(self.intervals) < 2:
            return 0.0
        mu = sum(self.intervals) / len(self.intervals)
        var = sum((x - mu) ** 2 for x in self.intervals) / (len(self.intervals) - 1)
        return math.sqrt(max(0.0, var))


class ImuOdomMonitor(Node):
    def __init__(self, imu_topic: str, odom_topic: str, window_sec: float,
                 warn_imu_rate: float, warn_odom_rate: float, log_csv: str | None):
        super().__init__('imu_odom_monitor')

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=50,
        )

        self.imu_stats = TopicStats(window_sec)
        self.odom_stats = TopicStats(window_sec)
        self.imu_acc_norms: Deque[float] = deque(maxlen=500)

        self.warn_imu_rate = warn_imu_rate
        self.warn_odom_rate = warn_odom_rate

        self.log_writer = None
        if log_csv:
            p = Path(log_csv).expanduser()
            if p.parent and not p.parent.exists():
                p.parent.mkdir(parents=True, exist_ok=True)
            self.log_file = p.open('w', newline='')
            self.log_writer = csv.writer(self.log_file)
            self.log_writer.writerow([
                't', 'imu_rate', 'imu_delay', 'imu_jitter', 'imu_acc_norm_mean',
                'odom_rate', 'odom_delay', 'odom_jitter'
            ])
        else:
            self.log_file = None

        self.create_subscription(Imu, imu_topic, self.imu_cb, qos)
        self.create_subscription(Odometry, odom_topic, self.odom_cb, qos)

        self.timer = self.create_timer(1.0, self.tick)
        self.get_logger().info(f"Monitoring IMU: {imu_topic}, Odom: {odom_topic}, window={window_sec}s")

    def now_sec(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    @staticmethod
    def hdr_sec(msg_header) -> float:
        return msg_header.stamp.sec + msg_header.stamp.nanosec * 1e-9

    def imu_cb(self, msg: Imu):
        now_s = self.now_sec()
        hdr_s = self.hdr_sec(msg.header)
        self.imu_stats.push(now_s, hdr_s)
        ax, ay, az = msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z
        self.imu_acc_norms.append(math.sqrt(ax*ax + ay*ay + az*az))

    def odom_cb(self, msg: Odometry):
        now_s = self.now_sec()
        hdr_s = self.hdr_sec(msg.header)
        self.odom_stats.push(now_s, hdr_s)

    def tick(self):
        imu_rate = self.imu_stats.rate()
        imu_delay = self.imu_stats.delay_mean()
        imu_jitter = self.imu_stats.jitter_std()
        acc_norm_mean = (sum(self.imu_acc_norms) / len(self.imu_acc_norms)) if self.imu_acc_norms else 0.0

        odom_rate = self.odom_stats.rate()
        odom_delay = self.odom_stats.delay_mean()
        odom_jitter = self.odom_stats.jitter_std()

        self.get_logger().info(
            ("IMU: rate={:.1f}Hz delay={:.3f}s jitter={:.3f}s | "
             "acc_norm≈{:.2f}m/s^2 || Odom: rate={:.1f}Hz delay={:.3f}s jitter={:.3f}s").format(
                imu_rate, imu_delay, imu_jitter, acc_norm_mean,
                odom_rate, odom_delay, odom_jitter
            )
        )

        if imu_rate > 0 and imu_rate < self.warn_imu_rate:
            self.get_logger().warn(f"IMU rate low: {imu_rate:.1f}Hz (<{self.warn_imu_rate})")
        if odom_rate > 0 and odom_rate < self.warn_odom_rate:
            self.get_logger().warn(f"Odom rate low: {odom_rate:.1f}Hz (<{self.warn_odom_rate})")

        if self.log_writer:
            t_s = self.now_sec()
            self.log_writer.writerow([
                f"{t_s:.3f}", f"{imu_rate:.3f}", f"{imu_delay:.4f}", f"{imu_jitter:.4f}", f"{acc_norm_mean:.3f}",
                f"{odom_rate:.3f}", f"{odom_delay:.4f}", f"{odom_jitter:.4f}"
            ])

    def destroy_node(self):
        if self.log_file:
            self.log_file.close()
        super().destroy_node()


def main():
    parser = argparse.ArgumentParser(description='IMU/Odom topic monitor')
    parser.add_argument('--imu-topic', default='/livox/imu', help='IMU topic (sensor_msgs/Imu)')
    parser.add_argument('--odom-topic', default='/slam/lio_odom', help='Odometry topic (nav_msgs/Odometry)')
    parser.add_argument('--window-sec', type=float, default=5.0, help='Sliding window seconds')
    parser.add_argument('--warn-imu-rate', type=float, default=150.0, help='Warn if IMU rate below this (Hz)')
    parser.add_argument('--warn-odom-rate', type=float, default=5.0, help='Warn if Odom rate below this (Hz)')
    parser.add_argument('--use-sim-time', action='store_true', help='Enable simulated time (bag replay)')
    parser.add_argument('--log-csv', default='', help='CSV file to log metrics (optional)')
    args = parser.parse_args()

    rclpy.init()
    node = ImuOdomMonitor(
        imu_topic=args.imu_topic,
        odom_topic=args.odom_topic,
        window_sec=args.window_sec,
        warn_imu_rate=args.warn_imu_rate,
        warn_odom_rate=args.warn_odom_rate,
        log_csv=(args.log_csv if args.log_csv else None),
    )
    if args.use_sim_time:
        try:
            node.set_parameters([rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])
        except Exception:
            pass

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
