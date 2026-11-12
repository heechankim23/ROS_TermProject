"""[시뮬레이션 전용] 카메라 + apriltag_ros 를 흉내 내는 검출기 (perception:=sim).

킥보드 실제 위치(/sim/scooter_truth)와 충전소 태그 위치(map_config)를 받아,
로봇 카메라 시야(FOV·거리) 안에 들어온 태그만 노이즈를 섞어 /tag_detections 로 발행한다.
VM 에서 카메라 렌더링이 느리거나 안 될 때도 전체 파이프라인을 돌리기 위한 대체 경로.
실제 카메라를 쓰려면 perception:=apriltag (apriltag_adapter) 로 바꾼다.
"""
import math
import random

import rclpy
import yaml
from rclpy.node import Node

from kickboard_interfaces.msg import ScooterTruthArray, TagDetection, TagDetectionArray

from .collector import tag_id_of
from .common import PoseTracker
from .gz_utils import SCOOTER_L, SCOOTER_W
from .motion import quat_from_yaw, wrap, yaw_from_quat


class SimTagDetector(Node):
    def __init__(self):
        super().__init__('sim_tag_detector')
        self.declare_parameter('map_config_file', '')
        self.declare_parameter('hfov_deg', 62.2)           # Waffle Pi 카메라
        self.declare_parameter('max_range', 3.0)
        self.declare_parameter('min_range', 0.12)
        self.declare_parameter('max_view_angle_deg', 70.0)  # 태그 면을 비스듬히 볼 수 있는 한계
        self.declare_parameter('camera_offset_x', 0.07)
        self.declare_parameter('noise_pos', 0.02)
        self.declare_parameter('noise_yaw_deg', 2.0)
        self.declare_parameter('rate_hz', 10.0)
        with open(self.get_parameter('map_config_file').value) as f:
            c = yaml.safe_load(f)['charger']
        self.charger_face = (c['tag_pose']['x'], c['tag_pose']['y'], math.radians(c['tag_pose']['yaw_deg']))
        self.charger_id = int(c['tag_id'])
        self.truth = []
        self.pose = PoseTracker(self)
        self.create_subscription(ScooterTruthArray, '/sim/scooter_truth', self.on_truth, 10)
        self.pub = self.create_publisher(TagDetectionArray, '/tag_detections', 10)
        self.create_timer(1.0 / self.get_parameter('rate_hz').value, self.tick)

    def on_truth(self, m):
        self.truth = [(sid, p.position.x, p.position.y, yaw_from_quat(p.orientation))
                      for sid, p in zip(m.ids, m.poses)]

    @staticmethod
    def faces(x, y, yaw):
        out = []
        for lx, ly, ny in ((SCOOTER_L / 2, 0, 0), (-SCOOTER_L / 2, 0, math.pi),
                           (0, SCOOTER_W / 2, math.pi / 2), (0, -SCOOTER_W / 2, -math.pi / 2)):
            c, s = math.cos(yaw), math.sin(yaw)
            out.append((x + c * lx - s * ly, y + s * lx + c * ly, yaw + ny))
        return out

    def visible(self, cam, face):
        fx, fy, fyaw = face
        vx, vy = cam[0] - fx, cam[1] - fy
        d = math.hypot(vx, vy)
        g = self.get_parameter
        if not (g('min_range').value <= d <= g('max_range').value):
            return None
        view = abs(wrap(math.atan2(vy, vx) - fyaw))                       # 태그 법선 vs 카메라 방향
        bearing = abs(wrap(math.atan2(fy - cam[1], fx - cam[0]) - cam[2]))  # 카메라 광축 vs 태그
        if view > math.radians(g('max_view_angle_deg').value) or bearing > math.radians(g('hfov_deg').value / 2):
            return None
        return d, view

    def detection(self, tag, face, d):
        n, ny = self.get_parameter('noise_pos').value, math.radians(self.get_parameter('noise_yaw_deg').value)
        t = TagDetection(id=tag, distance=float(d))
        t.pose.position.x = face[0] + random.gauss(0, n)
        t.pose.position.y = face[1] + random.gauss(0, n)
        t.pose.position.z = 0.25
        q = quat_from_yaw(face[2] + random.gauss(0, ny))
        t.pose.orientation.x, t.pose.orientation.y, t.pose.orientation.z, t.pose.orientation.w = q
        return t

    def tick(self):
        rp = self.pose.get()
        if rp is None:
            return
        off = self.get_parameter('camera_offset_x').value
        cam = (rp[0] + off * math.cos(rp[2]), rp[1] + off * math.sin(rp[2]), rp[2])
        out = TagDetectionArray()
        out.header.frame_id = 'map'
        out.header.stamp = self.get_clock().now().to_msg()
        for sid, x, y, yaw in self.truth:
            best = None
            for f in self.faces(x, y, yaw):
                v = self.visible(cam, f)
                if v and (best is None or v[1] < best[1][1]):
                    best = (f, v)
            if best:
                out.detections.append(self.detection(tag_id_of(sid), best[0], best[1][0]))
        v = self.visible(cam, self.charger_face)
        if v:
            out.detections.append(self.detection(self.charger_id, self.charger_face, v[0]))
        self.pub.publish(out)


def main():
    rclpy.init()
    node = SimTagDetector()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
