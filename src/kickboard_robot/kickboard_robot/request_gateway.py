"""F2: 수거 요청 접수. 검증 -> GPS 를 map 좌표로 변환 -> queue_manager 에 전달 -> 순번 응답."""
import math

import rclpy
import yaml
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node

from kickboard_interfaces.srv import EnqueueRequest, RequestCollection

from .common import EventLog, spin_single
from .geo import GeoConverter


class RequestGateway(Node):
    def __init__(self):
        super().__init__('request_gateway')
        self.declare_parameter('map_config_file', '')
        self.declare_parameter('max_range_m', 500.0)       # 기준점에서 이보다 멀면 거절
        with open(self.get_parameter('map_config_file').value) as f:
            g = yaml.safe_load(f)['gps_origin']
        self.geo = GeoConverter(g['lat'], g['lon'])
        self.ev = EventLog(self)
        # on_request 는 코루틴: queue_manager 응답을 await 하는 동안 executor 가 다른 콜백을 처리한다.
        # 주의: 서비스와 클라이언트가 기본(MutuallyExclusive) 그룹에 있으면, await 중인 서비스가 그룹을 붙잡아
        # 클라이언트 응답이 처리되지 못하고 교착된다 -> Reentrant 그룹 필수
        cb = ReentrantCallbackGroup()
        self.cli = self.create_client(EnqueueRequest, '/queue/enqueue', callback_group=cb)
        self.create_service(RequestCollection, '/request_collection', self.on_request, callback_group=cb)

    async def on_request(self, req, res):
        if not req.scooter_id:
            res.accepted, res.message = False, 'scooter_id 비어 있음'
            return res
        if req.reason not in (RequestCollection.Request.FALLEN, RequestCollection.Request.OUT_OF_ZONE,
                              RequestCollection.Request.LOW_BATTERY):
            res.accepted, res.message = False, f'알 수 없는 reason {req.reason}'
            return res
        if not (-90 <= req.lat <= 90 and -180 <= req.lon <= 180):
            res.accepted, res.message = False, '위경도 범위 오류'
            return res
        x, y = self.geo.to_map(req.lat, req.lon)
        if math.hypot(x, y) > self.get_parameter('max_range_m').value:
            res.accepted, res.message = False, f'서비스 지역 밖 ({x:.0f}, {y:.0f})'
            return res
        if not self.cli.service_is_ready():
            res.accepted, res.message = False, 'queue_manager 응답 없음'
            return res
        q = EnqueueRequest.Request(scooter_id=req.scooter_id, reason=req.reason,
                                   x=x, y=y, lat=req.lat, lon=req.lon)
        r = await self.cli.call_async(q)
        if r is None:
            res.accepted, res.message = False, 'queue_manager 타임아웃'
            return res
        res.accepted, res.queue_position = r.accepted, r.queue_position
        res.message = ('병합됨: ' if r.merged else '') + r.message
        self.ev.emit('request_received', id=req.scooter_id, reason=req.reason, x=x, y=y,
                     merged=r.merged, position=r.queue_position)
        return res


def main():
    rclpy.init()
    spin_single(RequestGateway())
