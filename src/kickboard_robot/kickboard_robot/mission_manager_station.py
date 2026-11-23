"""킥보드 충전소가 '별도 장소'에 있는 버전의 미션 관리자 (mission_manager 의 서브클래스).

기본판(mission_manager.py): 배터리 부족 킥보드를 로봇 충전 도크 양옆 충전 자리에 내려놓는다 (도킹 직후).
이 판                     : 맵 설정의 scooter_charging_station(킥보드 충전소)까지 싣고 가서 내려놓는다.
                            로봇은 그 뒤에 자기 도크로 돌아가 충전한다.

  수거 -> (배터리 정상) 주차구역 하역 -> (배터리 부족) 킥보드 충전소로 이동·하역 -> 로봇 도크 복귀 -> 로봇 충전

- 충전소도 주차구역처럼 '한 변이 차도에 닿는 사각형'이다. 로봇은 그 변 밖(차도)에 서서 로봇팔로 충전 자리(4곳)에 세운다.
- 자리가 다 차 있으면 가장 오래 맡긴 킥보드를 '충전 끝'으로 치우고 그 자리를 쓴다 (기본판과 같은 규칙).
- 로봇 배터리가 부족하면 킥보드를 실은 채 먼저 도크에서 충전하고, 충전 뒤에 충전소로 간다.
- 충전소로 가는 길이 없으면 관리자 알림을 내고 도크로 돌아가 적재함을 비운다 (사람이 처리한다고 가정).
상태 이름은 기본판 그대로 쓴다 (충전소 하역도 DROPPING) -> 다른 노드·메시지를 바꾸지 않아도 된다.
mission_manager.py 는 고치지 않았다. 맵 설정에 `mission_variant: station` 이 있으면 launch 가 이 노드를 띄운다.
"""
import yaml
from geometry_msgs.msg import PoseStamped

import rclpy

from kickboard_interfaces.action import GoToPose
from kickboard_interfaces.srv import SelectNextTarget

from .common import spin_single
from .drop_zones import edge_plans
from .mission_manager import CHARGER, CHARGING, DROPPING, IDLE, PARKING, SELECTING, MissionManager
from .motion import quat_from_yaw
from .occupancy import OccupancyMap


class StationMissionManager(MissionManager):
    def load_drop_plans(self):
        self.station = None                       # {'zone', 'robot': (x, y, yaw), 'spots': [...]}
        plans = super().load_drop_plans()         # 주차구역 + (도크 옆 충전 자리: 아래에서 충전소 자리로 바꾼다)
        if not self.p('map_config_file'):
            return plans
        with open(self.p('map_config_file')) as f:
            st = (yaml.safe_load(f) or {}).get('scooter_charging_station')
        if not st:
            self.get_logger().warn('맵 설정에 scooter_charging_station 이 없음 -> 기본판처럼 도크에서 하역')
            return plans
        omap = OccupancyMap(self.p('map_yaml')) if self.p('map_yaml') else None
        cand = edge_plans(st.get('name', 'CS'), st['polygon'], omap, self.p('drop_clearance'))
        if not cand:
            self.get_logger().warn('킥보드 충전소 앞에 로봇이 설 자리가 없음 -> 기본판처럼 도크에서 하역')
            return plans
        self.station = cand[0]
        self.charger_spots = list(self.station['spots'])      # 충전 자리 = 충전소 안
        self.charger_occ = [None] * len(self.charger_spots)
        self.charger_fifo = []
        rx, ry, _ = self.station['robot']
        self.get_logger().info(f'킥보드 충전소 {self.station["zone"]}: 로봇 위치 ({rx:.2f}, {ry:.2f}), '
                               f'충전 자리 {len(self.charger_spots)}개 (로봇 도크와 별도 장소)')
        return plans

    # ---------------- 판단 ----------------
    def do_select(self):
        if self.station is None:
            return super().do_select()
        if self.at_dock and self.dock_unload_all and self.cargo:       # 충전소·주차구역에 못 간 경우만 도크에서 비움
            self.unload_at_dock(lambda n: self.set_state(SELECTING, f'(도크에서 적재함 비움 {n}대)'))
            return
        low = self.battery <= self.p('return_battery_pct')
        if low or self.load >= self.p('capacity'):
            why = f'(배터리 {self.battery:.0f}%)' if low else '(만재)'
            if not low and self.start_any_drop(why):
                return
            if self.at_dock:
                self.set_state(CHARGING, why)
            else:
                self.start_dock(why)
            return
        pose = self.pose.get()
        if pose is None:
            return
        self.pending = True
        fut = self.select_cli.call_async(SelectNextTarget.Request(robot_x=pose[0], robot_y=pose[1]))
        fut.add_done_callback(self.on_selected)

    def on_selected(self, fut):
        if self.station is None:
            return super().on_selected(fut)
        self.pending = False
        r = fut.result()
        if r is None:
            return
        if r.found:
            self.start_collect(r.scooter_id, r.target_pose)
            return
        if self.start_any_drop('(대기열 빈)'):
            return
        if self.at_dock and not self.cargo:
            self.set_state(IDLE, '(대기열 빈)')
        elif self.at_dock:
            self.set_state(SELECTING)             # dock_unload_all 로 다음 tick 에 도크에서 비운다
        else:
            self.start_dock('(대기열 빈)')

    def start_any_drop(self, why):
        """주차구역행이 있으면 주차구역, 그다음 충전소행이 있으면 킥보드 충전소. 갈 곳을 정했으면 True"""
        if self.dock_unload_all:
            return False
        if self.cargo_for(PARKING) and self.start_drop(why):
            return True
        if self.dock_unload_all:                  # 주차구역에 자리가 없었다 -> 도크에서 전부 하역
            return False
        if self.cargo_for(CHARGER) and self.start_station_drop(why):
            return True
        return False

    # ---------------- 킥보드 충전소 하역 ----------------
    def start_station_drop(self, why):
        ids = self.cargo_for(CHARGER)
        rx, ry, ryaw = self.station['robot']
        self.at_dock = False
        self.set_state(DROPPING, f'{why} -> 킥보드 충전소 {self.station["zone"]} ({len(ids)}대)')
        target = PoseStamped()
        target.header.frame_id = 'map'
        target.pose.position.x, target.pose.position.y = rx, ry
        o = target.pose.orientation
        o.x, o.y, o.z, o.w = quat_from_yaw(ryaw)
        self.pending = True
        f = self.nav_ac.send_goal_async(GoToPose.Goal(target_pose=target, standoff=0.0))
        f.add_done_callback(self.on_station_nav_accepted)
        return True

    def on_station_nav_accepted(self, f):
        h = f.result()
        if h is None or not h.accepted:
            self.on_station_nav_failed('nav goal rejected')
            return
        h.get_result_async().add_done_callback(self.on_station_nav_done)

    def on_station_nav_done(self, f):
        r = f.result()
        if r is None or not r.result.success:
            self.on_station_nav_failed(r.result.message if r is not None else 'no result')
            return
        name = self.station['zone']
        self.ev.emit('arrived_at_station', station=name, count=len(self.cargo_for(CHARGER)))
        # 충전 자리 배정·오래된 킥보드 치우기는 기본판의 도크 하역 절차를 그대로 쓴다 (자리만 충전소 것)
        MissionManager.unload_at_dock(
            self, lambda n: self.set_state(SELECTING, f'(킥보드 충전소 {name} 하역 {n}대)'))

    def on_station_nav_failed(self, why):
        self.pending = False
        self.alert('station', 'nav_failed', f'킥보드 충전소 이동 실패: {why} -> 도크에서 적재함을 비움')
        self.dock_unload_all = True
        self.start_dock('(킥보드 충전소 이동 실패)')

    # ---------------- 도크 ----------------
    def unload_at_dock(self, then):
        """도킹 뒤 호출된다. 이 판에서는 도크에 킥보드를 내려놓지 않는다 (충전소·주차구역에 못 간 경우만 적재함을 비움)."""
        if self.station is None:
            return super().unload_at_dock(then)
        if self.dock_unload_all and self.cargo:
            spots, self.charger_spots = self.charger_spots, []       # 자리 없음 -> 기본판이 '적재함에서 바로 치움'으로 처리
            super().unload_at_dock(then)
            self.charger_spots = spots
            return
        self.dock_unload_all = False
        self.run_unloads([], then)


def main():
    rclpy.init()
    spin_single(StationMissionManager())
