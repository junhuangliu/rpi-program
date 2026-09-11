#!/usr/bin/env python3
"""前方矩形区域障碍物检测服务节点。

功能：
  1. 订阅 /scan（sensor_msgs/LaserScan），缓存最新一帧雷达数据
  2. 提供 ROS2 Service /obstacle/check（rpi_msgs/srv/ObstacleCheck）：
     检查机器人前方矩形区域内（base_link 系：x=正前方，y=左右）有无障碍物；
     矩形内点按扫描顺序相邻点欧氏距离聚类，每个障碍物簇返回中心坐标等
  3. 坐标约定：返回坐标一律在 base_link 系（前方 = +x），
     `/scan` 为激光系，由 radar_inverted（绕 z 转 180°，雷达逆装朝后，
     与 slam_launch.py 中 base_link->laser 静态 TF yaw=π 一致）换算
  4. 固定矩形区域（无请求参数）：x∈[0.2, 1.0]m、y∈[-0.15, 0.15]m，
     与 rplidar 屏蔽最近距离 0.20m 衔接；可用节点参数覆盖

用法：
  由 slam_launch.py 拉起（随 slam 启停），也可单独运行：
  python3 start.py obstacle

  调用示例：
    ros2 service call /obstacle/check rpi_msgs/srv/ObstacleCheck "{}"
"""

import argparse
import math
import sys
import threading

try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import LaserScan
    from rpi_msgs.msg import Obstacle
    from rpi_msgs.srv import ObstacleCheck

    _ROS_ERR = None
except ModuleNotFoundError as e:
    _ROS_ERR = e

SCAN_TOPIC = "scan"
SERVICE_NAME = "obstacle/check"
RADAR_INVERTED = True      # 雷达逆装朝后：激光系绕 z 转 180° 即 base_link 系
DEFAULT_RECT = {
    "x_min": 0.2,
    "x_max": 1.0,
    "y_min": -0.15,
    "y_max": 0.15,
}
CLUSTER_THRESH = 0.15    # 同一障碍物内相邻点最大间隔(m)
CACHE_TIMEOUT = 1.0      # 超过该秒数的扫描视为过期


class ObstacleCheckNode(Node):
    """障碍物检测节点：缓存最新 /scan，服务回调时聚类并返回障碍物坐标。"""

    def __init__(self) -> None:
        super().__init__("obstacle_check")
        for name, default in DEFAULT_RECT.items():
            self.declare_parameter(name, default)
        self.declare_parameter("cluster_distance_threshold", CLUSTER_THRESH)
        self._rect = {
            name: float(self.get_parameter(name).value) for name in DEFAULT_RECT
        }
        self._clust_thresh = float(
            self.get_parameter("cluster_distance_threshold").value)

        self._scan = None
        self._scan_stamp = 0.0
        self._lock = threading.Lock()
        self._sub = self.create_subscription(
            LaserScan, SCAN_TOPIC, self.on_scan, 10)
        self._srv = self.create_service(
            ObstacleCheck, SERVICE_NAME, self.handle_check)
        self.get_logger().info(
            f"矩形区域: x=[{self._rect['x_min']},{self._rect['x_max']}], "
            f"y=[{self._rect['y_min']},{self._rect['y_max']}], "
            f"聚类阈值={self._clust_thresh}m")
        self.get_logger().info(
            f"服务: /{SERVICE_NAME}  "
            f"(调用: ros2 service call /{SERVICE_NAME} rpi_msgs/srv/ObstacleCheck \"{{}}\")")

    def on_scan(self, msg: LaserScan) -> None:
        """缓存最新一帧扫描（无额外计算，聚类在服务回调时做）。"""
        with self._lock:
            self._scan = msg
            self._scan_stamp = self.get_clock().now().nanoseconds * 1e-9

    def handle_check(self, request, response):
        """服务回调：矩形内点聚类（base_link 系，前方=+x），填充障碍物坐标列表。"""
        with self._lock:
            scan = self._scan
            if scan is None:
                response.success = False
                response.has_obstacle = False
                response.obstacles = []
                response.min_distance = -1.0
                self.get_logger().info("无 /scan 数据，无法检测")
                return response
            stamp = self._scan_stamp

        age = self.get_clock().now().nanoseconds * 1e-9 - stamp
        if age > CACHE_TIMEOUT:
            response.success = False
            response.has_obstacle = False
            response.obstacles = []
            response.min_distance = -1.0
            self.get_logger().info(f"扫描数据过期({age:.1f}s)，无法检测")
            return response

        points = self._collect_points(scan)
        clusters = self._cluster(points)
        response.success = True
        response.has_obstacle = bool(clusters)
        response.obstacles = [self._to_obstacle(c) for c in clusters]
        response.min_distance = (
            min(p[2] for c in clusters for p in c) if clusters else -1.0)
        self.get_logger().info(
            f"检测到 {len(clusters)} 个障碍物, "
            f"最近距离={response.min_distance:.3f}m")
        for o in response.obstacles:
            self.get_logger().info(
                f" 障碍物: x={o.x:.3f}, y={o.y:.3f}, "
                f"dist={o.distance:.3f}m, angle={math.degrees(o.angle):.1f}°, "
                f"点数={o.point_count}, nearest={o.nearest:.3f}m")
        return response

    def _collect_points(self, scan: LaserScan) -> list:
        """取 base_link 系 (x, y, distance, angle) 按扫描顺序，筛出矩形内的点。

        激光系有效点 (xl, yl) 绕 z 转 180° → base_link 系 (x, y) = (-xl, -yl)，
        与 slam_launch.py 的 base_link->laser 静态 TF( yaw=π ) 保持一致。
        """
        points = []
        angle = scan.angle_min
        for r in scan.ranges:
            if math.isfinite(r) and scan.range_min <= r <= scan.range_max:
                x = -r * math.cos(angle)
                y = -r * math.sin(angle)
                if (self._rect["x_min"] <= x <= self._rect["x_max"] and
                        self._rect["y_min"] <= y <= self._rect["y_max"]):
                    points.append((x, y, r, angle))
            angle += scan.angle_increment
        return points

    def _cluster(self, points: list) -> list:
        """相邻点欧氏距离 <= 阈值则同簇，返回簇列表（每簇为点列表）。"""
        clusters = []
        for p in points:
            if not clusters:
                clusters.append([p])
                continue
            if math.hypot(p[0] - clusters[-1][-1][0],
                          p[1] - clusters[-1][-1][1]) <= self._clust_thresh:
                clusters[-1].append(p)
            else:
                clusters.append([p])
        return clusters

    @staticmethod
    def _to_obstacle(cluster: list) -> Obstacle:
        """簇 → 障碍物消息：中心点均值 + 簇内最近点距离。"""
        n = len(cluster)
        avg_x = sum(p[0] for p in cluster) / n
        avg_y = sum(p[1] for p in cluster) / n
        closest = min(cluster, key=lambda p: p[2])
        o = Obstacle()
        o.x = avg_x
        o.y = avg_y
        o.distance = math.hypot(avg_x, avg_y)
        o.angle = math.atan2(avg_y, avg_x)
        o.point_count = n
        o.nearest = closest[2]
        return o


def main() -> None:
    if _ROS_ERR is not None:
        print(f"[错误] ROS2 Python 环境不可用: {_ROS_ERR}")
        print("  请先加载 ROS 环境，例如:")
        print("  source /opt/ros/jazzy/setup.bash && source ~/ros2_ws/install/setup.bash")
        sys.exit(1)

    parser = argparse.ArgumentParser(description="前方矩形区域障碍物检测服务节点")
    parser.parse_known_args()

    rclpy.init()
    node = ObstacleCheckNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("收到 Ctrl+C，退出")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()