"""노드 공용 유틸: TF 로 로봇 pose 얻기, 이벤트 로그 발행, sim-time 안전한 대기."""
import json
import math
import time

import rclpy
from rclpy.time import Time
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener

from .motion import yaw_from_quat


class PoseTracker:
    def __init__(self, node, map_frame='map', base_frame='base_footprint'):
        self.node = node
        self.map_frame, self.base_frame = map_frame, base_frame
        self.buf = Buffer()
        self.listener = TransformListener(self.buf, node, spin_thread=False)

    def get(self):
        """(x, y, yaw) in map, 없으면 None"""
        try:
            t = self.buf.lookup_transform(self.map_frame, self.base_frame, Time())
        except Exception:  # noqa: BLE001  (LookupException, ExtrapolationException ...)
            return None
        tr = t.transform
        return tr.translation.x, tr.translation.y, yaw_from_quat(tr.rotation)


class EventLog:
    """/mission_events (std_msgs/String, JSON) — rosbag 으로 기록해 tools/analyze 로 지표 계산."""

    def __init__(self, node):
        self.node = node
        self.pub = node.create_publisher(String, '/mission_events', 50)

    def emit(self, ev, **kw):
        kw['ev'] = ev
        kw['src'] = self.node.get_name()
        kw['t'] = now_sec(self.node)
        self.pub.publish(String(data=json.dumps(kw, ensure_ascii=False)))


def now_sec(node):
    return node.get_clock().now().nanoseconds * 1e-9


def wait_for(node, predicate, timeout_sec, poll=0.05):
    """sim time 기준 timeout. 별도 executor 스레드가 콜백을 처리하고 있어야 한다."""
    t0 = now_sec(node)
    while rclpy.ok():
        if predicate():
            return True
        if timeout_sec is not None and now_sec(node) - t0 > timeout_sec:
            return False
        time.sleep(poll)
    return False


def sleep_sim(node, sec, poll=0.02):
    t0 = now_sec(node)
    while rclpy.ok() and now_sec(node) - t0 < sec:
        time.sleep(poll)


def dist2d(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def run_blocking(fn, *args):
    """오래 걸리는 함수(주행 루프 등)를 별도 스레드에서 실행하고 await 가능한 Future 를 돌려준다.
    Action 서버의 execute 를 `async def` 로 두고 이걸 await 하면, executor 는 단일 스레드로도
    다른 콜백(TF, 검출, 액션 클라이언트 응답)을 계속 처리한다."""
    import threading
    from rclpy.task import Future
    fut = Future()

    def work():
        try:
            fut.set_result(fn(*args))
        except Exception as e:  # noqa: BLE001
            fut.set_exception(e)
    threading.Thread(target=work, daemon=True).start()
    return fut


def spin_single(node):
    """블로킹 콜백이 없는 노드용. rclpy 의 MultiThreadedExecutor 는 메시지마다 오버헤드가 커서
    (/clock 100 Hz 만으로 노드당 CPU 20~40%) 가능하면 이쪽을 쓴다."""
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    except Exception:  # noqa: BLE001
        if rclpy.ok():
            raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def future_wait(node, future, timeout_sec):
    """다른 스레드에서 처리되는 future 를 기다린다."""
    ok = wait_for(node, future.done, timeout_sec)
    return future.result() if ok else None


