"""경북대 캠퍼스 맵의 미션 관리자 (mission_manager 의 서브클래스, 기존 파일은 고치지 않음).

맵 설정(config/maps/knu_campus*.yaml)에 `mission_variant: campus` 가 있으면 launch 가 이 노드를 mission_manager 이름으로 띄운다.

기본판과 다른 점
  - 하역 장소가 '주차구역 변'이 아니라 거치대 슬롯이다. 실을 때 킥보드마다 종류를 정한다 (/scooter_states):
        고장 코드 있음         -> broken  (일반 구역 고장 분류 칸, 반대 방향)   * 고장 킥보드는 배터리가 부족해도 충전하지 않는다
        배터리 < 20 %          -> charge  (충전 주차구역 충전 스탠드)
        그 외                  -> normal  (일반 구역 정상 슬롯)
  - 칸 배정은 하역하러 갈 때 한다: 거치대 센서(/zone_occupancy)로 빈 칸을 보고, 로봇에서 가까운 구역부터.
        충전 칸이 하나도 없으면 일반 슬롯에 세우고 '충전 대기'로 관리자 알림.
        충전 스탠드 수는 시나리오 overrides.charging_capacity 로 줄일 수 있다 (앞 번호 칸부터 사용).
  - SCOOTOVER v19 하차: 주차구역은 뒤 램프, 충전 거치대는 앞 램프. 한 대씩 내린다 (arm_loader 하차 애니메이션) -> 칸마다 거치대를 마주 보고
    슬롯 중심에서 STOP_DIST(1.6 m) 앞, 그 킥보드가 실린 레인(왼 y +0.27 / 오른 y -0.27)이 슬롯 줄에 오도록 옆으로 비켜 선다.
    레인은 arm_loader 와 같은 규칙(실을 때 빈 레인 중 왼쪽부터)으로 따라간다 (self.lanes).
  - 하역하러 가는 때: 적재함이 찼을 때, 대기열이 비었을 때. 로봇 배터리가 부족하면 실은 채 도크에서 충전부터 한다.
  - 돌발 이벤트 (/sim/campus_events): stand_fault(그 칸 안 씀) · zone_closed(구역 일시 폐쇄) ·
    robot_fault(arm, second_load: 2번째 적재 중 로봇팔 고장 -> 대상을 대기열에 되돌리고 도크에서 정비 대기, 이후 수거 중지)
도크에서는 킥보드를 내리지 않는다 (기본판의 '충전소 하역' 없음).
"""
import json
import math

import rclpy
import yaml
from geometry_msgs.msg import PoseStamped
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String

from kickboard_interfaces.action import GoToPose
from kickboard_interfaces.msg import ScooterState
from kickboard_interfaces.srv import SelectNextTarget

from .common import now_sec, spin_single
from .collector_logic import reverse_entry_pose
from .mission_logic import CHARGING, DROPPING, IDLE, SELECTING   # 상태 이름은 D 의 mission_logic 에 있음
from .mission_manager import MissionManager
from .motion import quat_from_yaw

STAND_STOP_DIST = 1.6    # 충전 칸 중심 -> 로봇 중심 (앞으로 내림, 거치대를 마주 봄). v19 하차 애니메이션 끝 킥보드 위치와 같음
LANE_Y = (0.27, -0.27)   # 적재 레인 (로봇 기준 y). arm_loader 레인 0, 1
KIND_ROLE = {'normal': 'normal', 'broken': 'broken', 'charge': 'charge', 'charge_wait': 'normal'}
KIND_NAME = {'normal': '일반 슬롯', 'broken': '고장 분류 칸', 'charge': '충전 스탠드', 'charge_wait': '일반 슬롯(충전 대기)'}


def unload_stop_pose(slot, on_stand, lane):
    """칸(중심 x, y, yaw) -> 하역할 때 로봇이 설 자리 (x, y, yaw).
    on_stand(충전 거치대): 거치대를 마주 보고 STAND_STOP_DIST 앞 -> 앞 램프로 밀어 꽂는다.
    그 외(주차구역): 킥보드와 같은 방향으로 칸 앞 2.07 m -> 뒤 램프로 내린다 (후진 적재 자리와 같음)."""
    if not on_stand:
        return reverse_entry_pose(slot['x'], slot['y'], slot['yaw'], lane)
    c, s = math.cos(slot['yaw']), math.sin(slot['yaw'])
    ly = LANE_Y[lane]
    return slot['x'] - c * STAND_STOP_DIST + s * ly, slot['y'] - s * STAND_STOP_DIST - c * ly, slot['yaw']


class CampusMissionManager(MissionManager):
    # ---------------- 준비 ----------------
    def load_drop_plans(self):
        self.zones = {}            # id -> {'type', 'site', 'out', 'axis', 'center', 'slots': [...]}
        self.kind = {}             # 실은 킥보드 id -> normal | broken | charge | charge_wait
        self.sstate = {}           # 최신 /scooter_states
        self.occ, self.dead, self.reserved = {}, set(), {}
        self.lanes = {}          # 실린 킥보드 -> 레인 (arm_loader 와 같은 규칙)
        self.closed = {}           # 구역 -> 폐쇄 끝 시각
        self.charge_cap = None
        self.halted = False        # 로봇 고장으로 수거 중지
        self.arm_fail = None       # 'second_load'
        self.stops = []            # 이번 하역 순회 [(구역, (x, y, yaw), [(id, 슬롯)])]
        self.alerted_wait = set()
        if not self.p('map_config_file'):
            self.get_logger().error('map_config_file 없음')
            return []
        with open(self.p('map_config_file')) as f:
            mc = yaml.safe_load(f)
        for typ, key in (('general', 'parking_zones'), ('charging', 'charging_zones')):
            for z in mc.get(key, []) or []:
                if not z.get('slots'):
                    continue
                ref = next(s for s in z['slots'] if s['role'] != 'broken')
                oy = math.radians(ref['yaw_deg'])
                out = (math.cos(oy), math.sin(oy))
                self.zones[z['name']] = {'type': typ, 'site': z.get('site', ''), 'out': out, 'axis': (-out[1], out[0]),
                                         'center': (z['rack']['x'], z['rack']['y']),
                                         'slots': [dict(s, yaw=math.radians(s['yaw_deg'])) for s in z['slots']]}
        latched = QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, '/sim/campus_events', self.on_campus_event, latched, callback_group=self.cb)
        self.create_subscription(String, '/zone_occupancy', self.on_occupancy, 10, callback_group=self.cb)
        self.create_subscription(ScooterState, '/scooter_states', lambda m: self.sstate.__setitem__(m.id, m), 50,
                                 callback_group=self.cb)
        ng = sum(1 for z in self.zones.values() if z['type'] == 'general')
        self.get_logger().info(f'캠퍼스 하역: 일반 구역 {ng}곳, 충전 구역 {len(self.zones) - ng}곳, '
                               f'슬롯 {sum(len(z["slots"]) for z in self.zones.values())}칸')
        return []

    def on_occupancy(self, m):
        try:
            d = json.loads(m.data)
        except ValueError:
            return
        self.occ = d.get('occupied', {})
        self.dead |= set(d.get('dead', []))

    def on_campus_event(self, m):
        try:
            e = json.loads(m.data)
        except ValueError:
            return
        t = e.get('type')
        if t == 'config':
            cap = (e.get('overrides') or {}).get('charging_capacity')
            if cap:
                self.charge_cap = int(cap)
                self.get_logger().info(f'충전 주차구역 스탠드 {self.charge_cap}칸으로 제한')
        elif t == 'stand_fault':
            self.dead.add(e['slot'])
            self.alert(e['slot'], 'stand_fault', f'충전 스탠드 {e["slot"]} 고장 -> 사용 안 함')
        elif t == 'zone_closed':
            self.closed[e['zone']] = now_sec(self) + float(e.get('duration', 3600))
            self.alert(e['zone'], 'zone_closed', f'{e["zone"]} 일시 폐쇄 ({e.get("note", "")})')
        elif t == 'robot_fault':
            self.arm_fail = e.get('trigger', 'now')
            if self.arm_fail == 'now':
                self.robot_broken('로봇팔 고장')
        self.ev.emit('campus_event', **{k: v for k, v in e.items() if k in ('type', 'slot', 'zone', 'part')})

    # ---------------- 슬롯 ----------------
    def usable(self, zid, slot):
        z = self.zones[zid]
        if self.closed.get(zid, 0) > now_sec(self) or slot['id'] in self.dead:
            return False
        if slot['id'] in self.occ or slot['id'] in self.reserved:
            return False
        if z['type'] == 'charging' and self.charge_cap is not None:
            if int(slot['id'].split('-')[1]) > self.charge_cap:
                return False
        return True

    def pick_slot(self, kind, xy):
        """kind 에 맞는 빈 칸 중 xy 에서 가장 가까운 구역의 칸 -> (구역, 슬롯) 또는 None"""
        role = KIND_ROLE[kind]
        best = None
        for zid, z in self.zones.items():
            if (role == 'charge') != (z['type'] == 'charging'):
                continue
            free = [s for s in z['slots'] if s['role'] == role and self.usable(zid, s)]
            if not free:
                continue
            d = math.dist(xy, z['center'])
            if best is None or d < best[0]:
                best = (d, zid, free[0])
        return None if best is None else best[1:]

    # ---------------- 판단 ----------------
    def tick(self):
        if not self.ready and not self.nav_ac.server_is_ready():
            return                  # 캠퍼스 nav_bridge 는 큰 경로 격자를 만드느라 시작이 느리다 -> 준비될 때까지 출발하지 않음
        if self.halted and self.state == IDLE:
            return
        super().tick()

    def do_select(self):
        if self.halted:
            if self.at_dock:
                self.set_state(IDLE, '(로봇 고장 — 정비 대기, 남은 요청은 대기열에 보류)')
            else:
                self.start_dock('(로봇 고장 — 도크로 복귀)')
            return
        low = self.battery <= self.p('return_battery_pct')
        if low:
            if self.at_dock:
                self.set_state(CHARGING, f'(배터리 {self.battery:.0f}%, 실은 {self.cargo.load}대는 충전 뒤 하역)')
            else:
                self.start_dock(f'(배터리 {self.battery:.0f}%)')
            return
        if self.cargo.load >= self.p('capacity'):
            if not self.start_campus_drop('(만재)'):
                self.start_dock('(하역할 칸 없음)')
            return
        pose = self.pose.get()
        if pose is None:
            return
        self.pending = True
        fut = self.select_cli.call_async(SelectNextTarget.Request(robot_x=pose[0], robot_y=pose[1]))
        fut.add_done_callback(self.on_selected)

    def on_selected(self, fut):
        self.pending = False
        r = fut.result()
        if r is None:
            return
        if r.found:
            st = self.sstate.get(r.scooter_id)
            if st is not None and st.in_parking_zone and not st.fault and st.tilt_deg <= 45 \
                    and st.battery_pct >= self.p('scooter_battery_threshold_pct'):
                # 신고됐지만 펌웨어·거치대 센서로 보면 슬롯에 바르게 서 있는 정상 킥보드 (오신고 · 그새 사용자가 정렬)
                self.report(r.scooter_id, 'DONE', 'no action: parked correctly in slot')
                self.ev.emit('no_action', id=r.scooter_id, why='슬롯에 정상 주차 확인 (오신고 또는 이미 정렬됨)')
                self.get_logger().info(f'킥보드 {r.scooter_id}: 슬롯에 정상 주차 확인 -> 조치 불필요로 종료')
                self.set_state(SELECTING, '(조치 불필요)')
                return
            self.start_collect(r.scooter_id, r.target_pose)
            return
        if self.cargo.load and self.start_campus_drop('(대기열 빈)'):
            return
        if self.at_dock:
            self.set_state(IDLE, '(대기열 빈)')
        else:
            self.start_dock('(대기열 빈)')

    # ---------------- 수거 ----------------
    def on_collect_fb(self, m):
        super().on_collect_fb(m)
        if self.arm_fail == 'second_load' and m.feedback.phase == 'LOADING' and self.cargo.load >= 1 and not self.halted:
            self.robot_broken('2번째 적재 중 로봇팔 관절 이상')
            if self.collect_handle is not None:
                self.collect_handle.cancel_goal_async()

    def robot_broken(self, why):
        self.halted = True
        self.arm_fail = None
        self.alert('robot', 'robot_fault', f'{why} -> 수거 중지, 정비 대기 (남은 요청 보류)')
        self.ev.emit('robot_fault', why=why, load=self.cargo.load)

    def on_collect_done(self, f):
        res = f.result().result
        sid = self.target
        if self.halted:
            self.collect_handle, self.target = None, None
            if res.success:
                self.cargo.add(sid, 'normal')
            else:
                self.report(sid, 'REQUEUE', 'robot fault')
            self.start_dock('(로봇 고장)')
            return
        if not res.success:
            return super().on_collect_done(f)
        st = self.sstate.get(sid)
        if st is not None and st.fault:
            kind = 'broken'
        elif st is not None and st.battery_pct < self.p('scooter_battery_threshold_pct'):
            kind = 'charge'
        else:
            kind = 'normal'
        self.collect_handle = None
        self.kind[sid] = kind
        self.cargo.add(sid, kind)
        self.report(sid, 'DONE', 'loaded', res.corrected_pose.pose)
        self.ev.emit('collected', id=sid, corrected_x=res.corrected_pose.pose.position.x,
                     corrected_y=res.corrected_pose.pose.position.y, load=self.cargo.load, dest=kind,
                     fault=st.fault if st else '', battery=round(st.battery_pct, 1) if st else None)
        self.target = None
        info = f'고장 {st.fault}' if kind == 'broken' else f'배터리 {st.battery_pct:.0f}%' if st else '상태 모름'
        self.set_state(SELECTING, f'(적재 {self.cargo.load}/{self.p("capacity")}, 킥보드 {sid} {info} -> {KIND_NAME[kind]})')

    # ---------------- 하역 ----------------
    def start_campus_drop(self, why):
        """실은 킥보드마다 칸을 배정하고, 구역을 가까운 순서로 돌며 내린다. 배정할 칸이 하나도 없으면 False"""
        pose = self.pose.get()
        if pose is None or not self.cargo.load:
            return False
        here = pose[:2]
        self.sync_lanes()
        plan = {}
        for sid, _ in self.cargo:
            kind = self.kind.get(sid, 'normal')
            pick = self.pick_slot(kind, here)
            if pick is None and kind == 'charge':          # 충전 칸이 모두 참 -> 일반 슬롯에 충전 대기
                pick = self.pick_slot('normal', here)
                if pick is not None:
                    self.kind[sid] = 'charge_wait'
                    if sid not in self.alerted_wait:
                        self.alerted_wait.add(sid)
                        self.alert(sid, 'charge_wait', f'충전 스탠드가 모두 참 -> {pick[0]} 일반 슬롯에 충전 대기')
            if pick is None:
                self.alert(sid, 'no_slot', f'{KIND_NAME[kind]} 빈 칸 없음 -> 적재 유지')
                continue
            zid, slot = pick
            self.reserved[slot['id']] = sid
            plan.setdefault(zid, []).append((sid, slot))
        if not plan:
            return False
        # 구역 순서: 지금 위치에서 가장 가까운 구역부터 차례로 (greedy)
        order, cur = [], here
        left = dict(plan)
        while left:
            zid = min(left, key=lambda z: math.dist(cur, self.zones[z]['center']))
            order.append(zid)
            cur = self.zones[zid]['center']
            del left[zid]
        self.stops = []
        for zid in order:
            z = self.zones[zid]
            ax, ay = z['axis']
            jobs = sorted(plan[zid], key=lambda js: js[1]['x'] * ax + js[1]['y'] * ay)
            for sid, slot in jobs:                          # 한 칸씩: 그 킥보드의 레인을 슬롯 줄에 맞춤
                pose = unload_stop_pose(slot, z['type'] == 'charging', self.lanes.get(sid, 0))
                self.stops.append((zid, pose, [(sid, slot)]))
        desc = ', '.join(f'{zid}({len(plan[zid])})' for zid in order)
        self.ev.emit('drop_plan', zones=order, assignments={sid: (slot['id'], self.kind.get(sid))
                                                             for zid in order for sid, slot in plan[zid]})
        self.at_dock = False
        self.set_state(DROPPING, f'{why} -> {desc}')
        self.next_stop()
        return True

    def sync_lanes(self):
        """내린 킥보드는 레인에서 빼고, 새로 실린 킥보드(실은 순서)는 빈 레인 중 앞 번호에 (arm_loader 와 같은 규칙)"""
        ids = [c[0] for c in self.cargo]
        self.lanes = {sid: ln for sid, ln in self.lanes.items() if sid in ids}
        for sid in ids:
            if sid not in self.lanes:
                free = [ln for ln in range(len(LANE_Y)) if ln not in self.lanes.values()]
                self.lanes[sid] = free[0] if free else 0

    def next_stop(self):
        if not self.stops:
            self.reserved.clear()
            self.pending = False
            self.set_state(SELECTING, '(하역 끝)')
            return
        zid, (x, y, yaw), grp = self.stops[0]
        target = PoseStamped()
        target.header.frame_id = 'map'
        target.pose.position.x, target.pose.position.y = x, y
        o = target.pose.orientation
        o.x, o.y, o.z, o.w = quat_from_yaw(yaw)
        self.pending = True
        self.get_logger().info(f'하역 자리로 이동: {zid} ({x:.1f}, {y:.1f}) — '
                               + ', '.join(f'{sid}->{s["id"]}' for sid, s in grp))
        # 충전소 안에서 출발하면 depart 가 먼저 똑바로 후진해 빠져나온 뒤 보냄
        self.depart(lambda: self.nav_ac.send_goal_async(GoToPose.Goal(target_pose=target, standoff=0.0))
                    .add_done_callback(self.on_stop_accepted))

    def on_stop_accepted(self, f):
        h = f.result()
        if h is None or not h.accepted:
            self.on_stop_failed('nav goal rejected')
            return
        h.get_result_async().add_done_callback(self.on_stop_arrived)

    def on_stop_arrived(self, f):
        r = f.result()
        if r is None or not r.result.success:
            self.on_stop_failed(r.result.message if r is not None else 'no result')
            return
        zid, _, grp = self.stops.pop(0)
        kind = 'stand' if self.zones[zid]['type'] == 'charging' else 'park'
        jobs = [(sid, (s['x'], s['y'], s['yaw']), kind) for sid, s in grp]
        for sid, s in grp:
            self.ev.emit('slot_assigned', id=sid, zone=zid, slot=s['id'], role=s['role'], kind=self.kind.get(sid))

        def then(n):
            for sid, s in grp:
                self.reserved.pop(s['id'], None)
                if not any(c[0] == sid for c in self.cargo):      # 하역 성공
                    self.occ[s['id']] = sid
                    self.ev.emit('slot_filled', id=sid, zone=zid, slot=s['id'], kind=self.kind.pop(sid, ''))
                else:                                              # 실패: 그 칸은 쓰지 않고 다음 하역 때 다시 배정
                    self.dead.add(s['id'])
                    self.cargo = [(c, self.kind.get(c, 'normal')) if c == sid else (c, d) for c, d in self.cargo]
            self.get_logger().info(f'{zid} 하역 {n}대')
            self.next_stop()
        self.run_unloads(jobs, then)

    def on_stop_failed(self, why):
        zid, _, grp = self.stops.pop(0)
        for sid, s in grp:
            self.reserved.pop(s['id'], None)
        self.alert(zid, 'nav_failed', f'하역 자리 이동 실패: {why} -> 이 구역은 건너뜀')
        self.closed[zid] = now_sec(self) + 300.0
        self.next_stop()

    # ---------------- 도크 ----------------
    def unload_at_dock(self, then):
        """캠퍼스판은 도크에 킥보드를 내리지 않는다 (충전만)"""
        self.dock_unload_all = False
        self.run_unloads([], then)


def main():
    rclpy.init()
    spin_single(CampusMissionManager())
