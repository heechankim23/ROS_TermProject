"""apriltag_ros 출력 -> /tag_detections (perception:=apriltag).

apriltag_ros(christianrauch) 는 /detections (apriltag_msgs/AprilTagDetectionArray) 와
TF 프레임 '<family>:<id>' 를 낸다. 여기서는 TF 로 map 기준 태그 pose 를 구해 우리 메시지로 바꾼다.
TODO(E): 태그 프레임의 z 축 방향을 실제로 확인하고 normal_axis_sign 을 맞출 것 (+1: z 가 태그 바깥쪽).
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener

from kickboard_interfaces.msg import TagDetection, TagDetectionArray


class AprilTagAdapter(Node):
    def __init__(self):
        super().__init__('apriltag_adapter')
        self.declare_parameter('family', 'tag36h11')
        self.declare_parameter('normal_axis_sign', -1.0)
        from apriltag_msgs.msg import AprilTagDetectionArray  # apriltag_ros 설치 시에만 필요
        self.buf = Buffer()
        TransformListener(self.buf, self)
        self.pub = self.create_publisher(TagDetectionArray, '/tag_detections', 10)
        self.create_subscription(AprilTagDetectionArray, '/detections', self.on_dets, 10)

    def on_dets(self, msg):
        out = TagDetectionArray()
        out.header.frame_id = 'map'
        out.header.stamp = msg.header.stamp
        fam = self.get_parameter('family').value
        sgn = self.get_parameter('normal_axis_sign').value
        for d in msg.detections:
            try:
                t = self.buf.lookup_transform('map', f'{fam}:{d.id}', Time())
            except Exception:  # noqa: BLE001
                continue
            q = t.transform.rotation
            # 태그 z 축을 map 에 투영 -> 바깥쪽 법선의 yaw
            zx = 2 * (q.x * q.z + q.w * q.y)
            zy = 2 * (q.y * q.z - q.w * q.x)
            yaw = math.atan2(sgn * zy, sgn * zx)
            det = TagDetection(id=int(d.id))
            det.pose.position.x = t.transform.translation.x
            det.pose.position.y = t.transform.translation.y
            det.pose.position.z = t.transform.translation.z
            det.pose.orientation.z, det.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
            out.detections.append(det)
        self.pub.publish(out)


def main():
    rclpy.init()
    node = AprilTagAdapter()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
