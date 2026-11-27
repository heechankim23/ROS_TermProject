"""노드 가공부(*_logic.py) 단위 테스트. ROS 없이 돈다: cd src/kickboard_robot && python3 -m pytest -q test/"""
import math
import random

import numpy as np

from kickboard_robot import battery_logic, collector_logic, dock_logic, gateway_logic, lidar_logic
from kickboard_robot import localizer_logic, mission_logic as ml, nav_logic, queue_book, queue_logic as ql, tag_sim_logic
from kickboard_robot.geo import GeoConverter
from kickboard_robot.planner import Costmap


# ---------------- battery ----------------
def test_battery_drains_by_distance_and_time_and_charges():
    b = battery_logic.BatteryModel(50.0, k1_pct_per_m=1.0, k2_pct_per_s=0.02, charge_rate_pct_per_s=2.0)
    assert b.update(0.0, 'NAVIGATING') == 50.0          # 첫 호출은 dt = 0
    b.add_odom(0.0, 0.0)
    b.add_odom(2.0, 0.0)                                # 리셋으로 보고 무시 (> 1 m)
    b.add_odom(2.5, 0.0)                                # 0.5 m
    assert abs(b.update(10.0, 'NAVIGATING') - (50.0 - 0.5 - 0.2)) < 1e-9
    assert b.update(20.0, 'IDLE') == b.pct              # IDLE: 변화 없음
    assert abs(b.update(25.0, 'CHARGING') - (49.3 + 10.0)) < 1e-9
    assert b.update(1000.0, 'CHARGING') == 100.0
    assert not b.is_low(20.0)


# ---------------- gateway ----------------
def test_gateway_checks_and_range():
    assert gateway_logic.check_request('', 1, 0, 0) == 'scooter_id 비어 있음'
    assert 'reason' in gateway_logic.check_request('1', 9, 0, 0)
    assert gateway_logic.check_request('1', 1, 91, 0) == '위경도 범위 오류'
    assert gateway_logic.check_request('1', 2, 37.5, 127.0) == ''
    geo = GeoConverter(37.5665, 126.9780)
    x, y, err = gateway_logic.to_map_checked(geo, 37.5665, 126.9780, 500.0)
    assert err == '' and abs(x) < 1e-6 and abs(y) < 1e-6
    _, _, err = gateway_logic.to_map_checked(geo, 37.60, 126.9780, 500.0)
    assert err.startswith('서비스 지역 밖')


# ---------------- collector ----------------
def test_collector_center_and_mean():
    assert collector_logic.tag_id_of('7') == 7
    assert 1 <= collector_logic.tag_id_of('abc') <= 90
    c = collector_logic.center_from_tag(1.0, 0.0, 0.0, 0.2)       # 태그 법선 +x -> 중심은 -x 쪽
    assert abs(c[0] - 0.8) < 1e-12 and c[1] == 0.0
    m = collector_logic.mean_center([(0, 0, 0.1), (2, 2, 0.2), (4, 4, 0.3)], last_n=2)
    assert m == (3.0, 3.0, 0.3)
    assert collector_logic.mean_center([]) is None
    assert collector_logic.arrived_by_tag(1.7, 1.6, 0.2) and not collector_logic.arrived_by_tag(1.9, 1.6, 0.2)


def test_extended_collector_reads_all_tag_faces():
    cl = collector_logic
    hd = 0.7
    for face in range(4):
        tid = cl.face_tag_id(17, face)
        base, f = cl.split_tag_id(tid)
        assert (base, f) == (17, face)
        a = hd + cl.FACE_NORMAL[face]
        fx, fy = 2.0 + cl.FACE_DEPTH[face] * math.cos(a), 5.0 + cl.FACE_DEPTH[face] * math.sin(a)
        cx, cy, h = cl.scooter_from_face(fx, fy, a, face)
        assert abs(cx - 2.0) < 1e-9 and abs(cy - 5.0) < 1e-9 and abs(cl.wrap_angle(h - hd)) < 1e-9


def test_collector_align_retry_same_radius():
    """정렬 재시도: 반경은 그대로, 각도만 바꾼 자리. 이미 시도한 각도 근처는 다시 고르지 않는다."""
    deg = math.degrees
    base = math.radians(30)
    c = collector_logic.retry_angles(base, [90, -90, 180], [base], 45)
    assert [round(deg(a)) for a in c] == [120, -60, -150]
    # 두 번째 자리(120°)도 실패 -> 120° 는 빼고 남은 순서대로
    c2 = collector_logic.retry_angles(base, [90, -90, 180], [base, c[0]], 45)
    assert [round(deg(a)) for a in c2] == [-60, -150]
    # 후보끼리도 겹치지 않음 (+170° 와 180° 는 10° 차이)
    c3 = collector_logic.retry_angles(0.0, [170, 180], [0.0], 45)
    assert [round(deg(a)) for a in c3] == [170]
    assert collector_logic.retry_angles(0.0, [10, -20], [0.0], 45) == []
    # 자리는 중심에서 반경 r, 방향은 중심을 바라봄
    for a in (0.0, 1.0, -2.5, math.pi):
        x, y, yaw = collector_logic.align_pose(2.0, -1.0, 1.35, a)
        assert abs(math.hypot(x - 2.0, y + 1.0) - 1.35) < 1e-9
        assert abs(collector_logic.wrap_angle(math.atan2(-1.0 - y, 2.0 - x) - yaw)) < 1e-9


def test_collector_align_next_step_wait_then_angle():
    """정렬 실패: 먼저 기다렸다 같은 자리(지나가는 장애물), 또 실패하면 각도 변경(고정 장애물)."""
    step = collector_logic.align_next_step
    assert step(False, 'ok', '', 5.0) == 'done'
    for s in ('timeout', 'rejected', 'canceled'):
        assert step(False, s, '', 5.0) == 'done'
    assert step(False, 'failed', 'nav failed after 1 attempts: sensor timeout', 5.0) == 'wait_same'
    assert step(True, 'failed', 'nav failed after 1 attempts: sensor timeout', 5.0) == 'change_angle'
    assert step(False, 'failed', 'no path', 0.0) == 'change_angle'          # align_wait_sec 0 = 바로 각도 변경
    assert step(False, 'failed', 'out of arm reach (2.10 m)', 5.0) == 'change_angle'   # 자리 선택 문제는 기다려도 같음


def test_collector_arm_reach_window():
    """팔이 닿는 거리: [dist - 0.25, dist + far]. 정렬 생략(far 0.15)은 적재 허용(far 0.45)보다 좁다."""
    near, skip, margin = collector_logic.ARM_REACH_NEAR, collector_logic.ARM_REACH_SKIP_FAR, 0.45
    assert collector_logic.within_reach(1.35, 1.35, near, skip)
    assert collector_logic.within_reach(1.10, 1.35, near, skip)
    assert not collector_logic.within_reach(1.09, 1.35, near, margin)      # 너무 가까움
    assert collector_logic.within_reach(1.70, 1.35, near, margin)          # 실패해도 적재
    assert not collector_logic.within_reach(1.70, 1.35, near, skip)        # 하지만 정렬 생략은 안 함
    assert not collector_logic.within_reach(1.81, 1.35, near, margin)      # 팔이 안 닿음 -> 재시도/실패


# ---------------- dock ----------------
CHARGER = {'tag_pose': {'x': -4.548, 'y': 0.0, 'yaw_deg': 0.0}, 'dock_distance': 1.0, 'pre_dock_distance': 2.0}


def test_dock_poses_and_errors():
    tag, dock, pre = dock_logic.charger_poses(CHARGER)
    assert abs(dock[0] + 3.548) < 1e-9 and abs(pre[0] + 2.548) < 1e-9 and abs(dock[2] - math.pi) < 1e-9
    pe, lat, ye = dock_logic.dock_errors((dock[0] + 0.03, dock[1] + 0.04, dock[2]), dock)
    assert abs(pe - 0.05) < 1e-9 and abs(abs(lat) - 0.04) < 1e-9 and abs(ye) < 1e-9
    ok, pe, ye = dock_logic.within_accept((dock[0] + 0.04, dock[1], dock[2] + math.radians(4)), dock)
    assert ok
    ok, _, _ = dock_logic.within_accept((dock[0] + 0.06, dock[1], dock[2]), dock)
    assert not ok


def test_dock_goal_filter_smooths_then_locks():
    tag, dock, _ = dock_logic.charger_poses(CHARGER)
    f = dock_logic.DockGoalFilter(dock, 1.0, alpha=0.15, lock_dist=0.10)
    g = f.add_detection(tag[0], tag[1] + 0.1, tag[2])              # 첫 검출은 그대로
    assert abs(g[1] - 0.1) < 1e-12 and abs(g[0] - dock[0]) < 1e-12
    g = f.add_detection(tag[0], tag[1], tag[2])                    # 다음은 15% 만 따라감
    assert abs(g[1] - 0.085) < 1e-12
    assert not f.check_lock((0.0, 0.0, 0.0))
    assert f.check_lock((g[0] + 0.05, g[1], 0.0)) and f.locked
    assert f.add_detection(tag[0], tag[1] + 1.0, tag[2]) == g      # 고정 뒤에는 무시
    assert not f.check_lock((g[0], g[1], 0.0))                     # 이미 고정: 다시 True 아님


# ---------------- localizer ----------------
def test_map_odom_filter_jumps_then_smooths():
    f = localizer_logic.MapOdomFilter(0.1)
    e = f.update((1.0, 0.0, 0.0), (0.0, 0.0, 0.0))
    assert e == (1.0, 0.0, 0.0)
    e = f.update((1.1, 0.0, 0.0), (0.0, 0.0, 0.0))                # 10 cm 차이 -> 10% 만
    assert abs(e[0] - 1.01) < 1e-12
    e = f.update((3.0, 0.0, 0.0), (0.0, 0.0, 0.0))                # 0.5 m 넘게 어긋나면 바로 맞춤
    assert e[0] == 3.0


# ---------------- sim tag detector ----------------
def test_tag_sim_sees_scooter_in_front_only():
    m = tag_sim_logic.CameraModel(noise_pos=0.0, noise_yaw_deg=0.0)
    cam = tag_sim_logic.camera_pose((0.0, 0.0, 0.0), 0.07)
    front = tag_sim_logic.detect(cam, [('3', 1.5, 0.0, math.pi)], None, 100, m, random.Random(0))
    assert len(front) == 1 and front[0][0] == 3 and abs(front[0][4] - (1.5 - 0.3 - 0.07)) < 1e-9
    behind = tag_sim_logic.detect(cam, [('3', -1.5, 0.0, 0.0)], None, 100, m, random.Random(0))
    assert behind == []
    far = tag_sim_logic.detect(cam, [('3', 5.0, 0.0, math.pi)], None, 100, m, random.Random(0))
    assert far == []
    charger = tag_sim_logic.detect(cam, [], (2.0, 0.0, math.pi), 100, m, random.Random(0))
    assert charger and charger[0][0] == 100


# ---------------- sim lidar ----------------
def test_lidar_hits_wall_and_scooter():
    g = np.zeros((40, 40), dtype=bool)          # 4 x 4 m, 0.1 m
    g[:, 30] = True                             # x = 3.0 m 벽
    sim = lidar_logic.LidarSim(g, 0.1, 0.0, 0.0, samples=4, range_min=0.12, range_max=3.5)
    r = sim.cast(1.0, 2.0, 0.0, sim.grid_with([], {}))
    assert abs(r[2] - 2.0) < 0.06                # 빔 2 = 0 rad (angles: -pi, -pi/2, 0, pi/2)
    r2 = sim.cast(1.0, 2.0, 0.0, sim.grid_with([('1', 2.0, 2.0, 0.0)], {}))
    assert r2[2] < 0.8                           # 킥보드 상자가 벽보다 먼저 맞음
    noisy = sim.add_noise(r, 0.0)
    assert noisy[2] == float(r[2])


# ---------------- nav ----------------
def test_filter_scan_masks_own_body_and_finds_front():
    ranges = [0.5, float('inf'), 1.2, 0.3]
    # 빔 각도: pi, -pi/2 (inf), 0, pi/2
    front_min, own, _ = nav_logic.filter_scan(ranges, math.pi, -math.pi / 2, 0.12, 0.8, (-0.92, 0.89, 0.5))
    assert front_min == 1.2
    assert own[0] and own[3] and not own[2]      # 뒤로 0.5 m (x=0.3), 옆 0.3 m -> 차체 안


def test_choose_goal_without_standoff_and_fallback():
    g, fb = nav_logic.choose_goal((1.0, 2.0), 0.5, (0, 0), 0.0, 1.1, 0.0)
    assert g == (1.0, 2.0, 0.5) and not fb
    cm = Costmap(0.0, 0.0, 4.0, 4.0, res=0.1)
    g, fb = nav_logic.choose_goal((2.0, 2.0), 0.0, (0.5, 2.0), 1.0, 0.5, 0.0, cm=cm)
    assert g is not None and not fb and abs(math.hypot(g[0] - 2.0, g[1] - 2.0) - 1.0) < 0.15


def test_path_follower_reaches_goal_on_empty_map():
    cm = Costmap(0.0, 0.0, 6.0, 6.0, res=0.1)
    f = nav_logic.PathFollower((4.0, 3.0, 0.0), nav_logic.FollowParams(), cm=cm)
    x, y, th, t = 1.0, 3.0, 0.0, 0.0
    for _ in range(4000):
        r = f.step((x, y, th), t)
        if r.kind == 'arrived':
            break
        assert r.kind in ('drive', 'again')
        if r.kind == 'drive':
            x += r.v * math.cos(th) * 0.05
            y += r.v * math.sin(th) * 0.05
            th += r.w * 0.05
            t += 0.05
    assert r.kind == 'arrived' and math.hypot(x - 4.0, y - 3.0) < 0.1


def test_path_follower_safety_stop_times_out():
    f = nav_logic.PathFollower((3.0, 0.0, 0.0), nav_logic.FollowParams(blocked_timeout_sec=1.0))   # 코스트맵 없음
    r = f.step((0.0, 0.0, 0.0), 0.0, front_min=0.1)
    assert r.kind == 'drive' and r.v == 0.0
    r = f.step((0.0, 0.0, 0.0), 1.5, front_min=0.1)
    assert r.kind == 'fail' and r.msg == 'blocked'


# ---------------- queue ----------------
def test_queue_book_merge_select_update_cancel():
    b, w = queue_book.QueueBook(), ql.Weights()
    assert b.enqueue('1', ql.OUT_OF_ZONE, 1.0, 0.0, 0, 0, 0.0) is False
    assert b.enqueue('2', ql.LOW_BATTERY, 5.0, 0.0, 0, 0, 0.0) is False
    assert b.enqueue('1', ql.FALLEN, 1.5, 0.0, 0, 0, 3.0) is True          # 병합: 사유·위치 갱신, 시각 유지
    assert b.items['1'].reason == ql.FALLEN and b.items['1'].x == 1.5 and b.items['1'].t_enqueued == 0.0
    it, s, n = b.select(10.0, (0.0, 0.0), w)
    assert it.id == '1' and it.status == ql.IN_PROGRESS and n == 2
    assert b.cancel('1') == (False, '처리 중인 항목은 취소할 수 없음(비선점)', it)
    ok, it = b.update('1', 'REQUEUE', 'nav_failed: blocked', (1.2, 0.1))
    assert ok and it.status == ql.WAITING and it.nav_failures == 1 and (it.x, it.y) == (1.2, 0.1)
    ok, _ = b.update('1', 'BOGUS')
    assert not ok
    assert b.auto_cancel('2', 0.0, True, 80.0, 45.0, 15.0) is b.items['2'] and b.items['2'].status == ql.CANCELLED
    assert b.auto_cancel('1', 90.0, True, 80.0, 45.0, 15.0) is None
    rows = b.rows(20.0, (0.0, 0.0), w)
    assert rows[0][0].id == '1' and rows[-1][0].status == ql.CANCELLED


# ---------------- mission ----------------
P = ml.MissionParams(capacity=4, return_battery_pct=20.0, charge_target_pct=80.0)


def test_mission_tick_transitions():
    assert ml.decide_tick(ml.SELECTING, 90, P, 0, 0, False, False).action == 'SELECT'
    d = ml.decide_tick(ml.IDLE, 15, P, 3, 0, False, False)
    assert (d.action, d.state) == ('SET', ml.CHARGING)
    d = ml.decide_tick(ml.IDLE, 90, P, 1, 0, False, False)
    assert (d.action, d.state) == ('SET', ml.SELECTING)
    assert ml.decide_tick(ml.IDLE, 90, P, 0, 0, False, False).action == 'NONE'
    assert ml.decide_tick(ml.NAVIGATING, 19, P, 0, 0, True, False).action == 'ABORT_COLLECT'
    assert ml.decide_tick(ml.NAVIGATING, 19, P, 0, 0, True, True).action == 'NONE'     # 이미 취소 요청함
    assert ml.decide_tick(ml.LOADING, 19, P, 0, 0, True, False).action == 'NONE'       # 적재 중엔 안 끊음
    assert ml.decide_tick(ml.FAILED, 90, P, 0, 0.5, False, False).action == 'NONE'
    assert ml.decide_tick(ml.FAILED, 90, P, 0, 1.5, False, False).state == ml.SELECTING
    assert ml.decide_tick(ml.CHARGING, 80, P, 0, 0, False, False).state == ml.SELECTING


def test_mission_select_and_queue_empty():
    c = ml.Cargo()
    assert ml.decide_select(True, c, 90, P).action == 'QUERY'
    for k in range(4):
        c.add(str(k), ml.PARKING if k < 3 else ml.CHARGER)
    d = ml.decide_select(True, c, 90, P)
    assert d.action == 'UNLOAD_AT_DOCK'                         # 도크에 있고 충전소행이 있음
    d = ml.decide_select(False, c, 90, P)
    assert (d.action, d.try_drop, d.fallback, d.why) == ('RETURN', True, 'DOCK', '(만재)')
    d = ml.decide_select(False, c, 10, P)
    assert d.try_drop is False and d.why == '(배터리 10%)'        # 배터리 부족: 충전부터
    d = ml.decide_queue_empty(True, ml.Cargo())
    assert (d.try_drop, d.fallback) == (False, ml.IDLE)
    assert ml.classify_failure('nav_failed: blocked', True) == (ml.NEEDS_HUMAN, 'nav_failed')
    assert ml.classify_failure('nav_failed: blocked', False) == (ml.FAILED, 'nav_failed')
    assert ml.classify_failure('verify_failed: tag not found', True) == (ml.FAILED, 'verify_failed')
    assert ml.drop_destination(10, 15) == ml.CHARGER and ml.drop_destination(None, 15) == ml.PARKING


def test_cargo_and_charger_slots():
    c = ml.Cargo()
    c.add('a', ml.PARKING)
    c.add('b', ml.CHARGER)
    c.add('c', ml.PARKING)
    assert ml.parking_jobs(c, [(0, 0, 0), (1, 1, 0)]) == [('c', (0, 0, 0), 'park'), ('a', (1, 1, 0), 'park')]
    c.redirect_to_charger('a')
    assert c.ids_for_dock(False) == ['a', 'b'] and c.ids_for_dock(True) == ['a', 'b', 'c']
    s = ml.ChargerSlots([(0, 0, 0), (1, 0, 0)])
    assert s.plan_unload(['x', 'y']) == [('y', (0, 0, 0), 'charger'), ('x', (1, 0, 0), 'charger')]
    jobs = s.plan_unload(['z'])                                  # 꽉 참 -> 가장 오래된 y 를 치움
    assert jobs == [('y', None, 'charged'), ('z', (0, 0, 0), 'charger')]
    s.release('z')
    assert s.occ == [None, 'x'] and s.fifo == [1]
    assert ml.ChargerSlots([]).plan_unload(['q']) == [('q', None, 'remove')]


def test_cargo_is_list_for_variant_nodes():
    """campus/station/extended 노드는 cargo 를 리스트로 다룬다: append, 반복, 빈 판정, 통째 교체."""
    c = ml.Cargo()
    assert not c and c.load == 0
    c.append(('1', 'normal'))                                    # campus: self.cargo.append((sid, kind))
    c.add('2', ml.PARKING)
    assert c and c.load == 2 and [s for s, _ in c] == ['1', '2'] and c.for_dest('normal') == ['1']
    c.remove('1')                                                # id 로 지움
    assert c == [('2', ml.PARKING)]
    c2 = ml.Cargo([(s, ml.CHARGER) for s, _ in c])               # 통째 교체 -> setter 가 Cargo 로 감쌈
    assert c2.for_dest(ml.CHARGER) == ['2']


def test_resolve_watch_needs_consecutive_normal_reports():
    """출발한 요청의 사유 해소: 정상 보고가 연속 confirm 번이어야 확정, 중간에 한 번 튀면 처음부터."""
    w = ml.ResolveWatch(confirm=2)
    assert not w.update(0, 90, False, 70)                        # 넘어짐
    assert not w.update(1, 0, True, 70)                          # 세워짐 1회
    assert w.update(2, 0, True, 70) and '바르게' in w.why        # 2회 연속 -> 확정
    w = ml.ResolveWatch(confirm=2)
    w.update(0, 0, True, 70)
    assert not w.update(1, 0, False, 70)                         # 구역 밖으로 튐 -> 처음부터
    assert not w.update(2, 0, True, 70)
    assert not ml.ResolveWatch(confirm=1).update(0, 0, True, 70, fault='brake')   # 고장은 해소 없음


def test_resolve_watch_low_battery_needs_charging_trend():
    """배터리 부족은 충전 칸에서 배터리가 꾸준히 오를 때만 해소 (창 20 s 의 80 % 이상 + 0.3 % 넘게 오름)."""
    w = ml.ResolveWatch(confirm=1, charge_window=20.0, charge_rise=0.3)
    assert not w.update(0, 0, True, 15.0)                        # 기준 미만, 아직 추세 모름
    for t in range(1, 16):
        assert not w.update(t, 0, True, 15.0 + 0.03 * t)         # 15 s: 창의 80 % 미만
    assert w.update(17, 0, True, 15.6) and '충전 중' in w.why
    w = ml.ResolveWatch(confirm=1)
    for t in range(0, 21):
        r = w.update(t, 0, True, 15.0 - 0.01 * t)                # 내려가기만 함 -> 충전 아님
    assert not r


def test_drop_station_walls_ignores_smeared_garage_walls():
    # 차고 (0, 0) 방향 0, 로봇은 차고 축 위 sx 1.3 에서 안쪽(+x)을 봄. 지도 번짐으로 옆벽 칸이 축에서 0.4 m 까지 들어옴
    st = (0.0, 0.0, 0.0)
    pose = (1.3, 0.0, 0.0)
    walls = [(bx, by) for bx in (-1.0, -1.3, -1.6) for by in (-0.4, 0.45, 0.6)]   # 로봇 기준: 뒤쪽 옆벽
    assert not ml.rear_clear(walls, 2.9)
    assert ml.rear_clear(ml.drop_station_walls(walls, pose, st), 2.9)
    # 문 가운데나 차고 밖의 물체는 그대로 막힘
    assert not ml.rear_clear(ml.drop_station_walls(walls + [(-1.2, 0.1)], pose, st), 2.9)
    assert not ml.rear_clear(ml.drop_station_walls([(-2.8, 0.45)], pose, st), 2.9)   # sx -1.5: 차고 밖


def test_arm_loader_unload_blend_far_from_origin():
    # 원점에서 280 m 떨어진 곳, 방향 8° 차이: 보정 크기는 실제 차이(0.14 m)여야 하고 u=1 에서 목표와 같아야 함
    from kickboard_robot.arm_loader import blend_about, compose, inverse, rz
    end = (np.array([-140.0, -242.0, 0.0]), rz(math.radians(80)))
    goal = (np.array([-140.1, -241.9, 0.0]), rz(math.radians(88)))
    corr = compose(goal, inverse(end))
    done = compose(blend_about(corr, end[0], 1.0), end)
    assert np.allclose(done[0], goal[0]) and np.allclose(done[1], goal[1])
    half = compose(blend_about(corr, end[0], 0.5), end)
    assert np.linalg.norm((half[0] - end[0])[:2]) < 0.15


def test_arm_loader_fallen_lift_sequence():
    """넘어진 킥보드: 팔이 쥐기 전에는 안 서고, 세우는 동안 원래 프레임은 멈추며, 팔 궤적이 원래 동작과 이어짐."""
    import os
    from kickboard_robot.arm_loader import LOAD_RANGE, Rig
    path = os.path.join(os.path.dirname(__file__), '..', '..', 'kickboard_sim', 'config', 'scootover_rig.json')
    rig = Rig(path, [0.42, 0.0, 0.127])
    start, grab, lift = (rig.fallen[k] for k in ('start', 'grab', 'lift'))
    for lane in (0, 1):
        frames = [rig.fallen_frame(lane, i) for i in range(LOAD_RANGE[1] + lift)]
        assert all(u == 0.0 for _, _, u in frames[:grab + 1])
        assert all(k == grab for k, _, _ in frames[grab:grab + lift + 1])
        assert frames[grab + lift][2] == 1.0 and frames[-1][0] == LOAD_RANGE[1] - 1
        q = []
        for k, arm, _ in frames:
            row = rig.Q[lane][k].copy()
            if arm is not None:
                row[rig.fallen_cols] = arm
            q.append(row)
        step = np.abs(np.diff(np.array(q), axis=0)).max(axis=1)
        orig = np.abs(np.diff(rig.Q[lane][:grab + 20], axis=0)).max()
        assert step[start - 1:grab + lift + 1].max() <= orig + 1e-6          # 끊김 없음


def test_rear_clear_checks_corridor_behind_body():
    """도크 후진: 차체 뒤끝(-0.92)에서 남은 거리 + 여유 안, 차체 폭 안에 점이 있으면 막힘."""
    assert ml.rear_clear([], 1.0)
    assert not ml.rear_clear([(-1.5, 0.0)], 1.0)                 # 바로 뒤 0.6 m
    assert ml.rear_clear([(-1.5, 0.8)], 1.0)                     # 옆으로 비껴 있음
    assert ml.rear_clear([(-2.5, 0.0)], 1.0, margin=0.3)          # 통로(-2.22) 밖
    assert ml.rear_clear([(-1.5, 0.0)], 0.0, margin=0.3)          # 다 물러났으면 -1.22 까지만 봄
    assert ml.rear_clear([(1.0, 0.0)], 1.0)                      # 앞쪽 점(충전소)은 상관없음


def test_scan_to_body_points_drops_own_body_and_max_range():
    """라이다(앞 0.8 m) 기준 거리 -> 로봇 기준 점. 측정 끝값과 차체 안 점은 뺀다."""
    import math as m
    pts = ml.scan_to_body_points([2.0, 3.5, 0.3], -m.pi / 2, m.pi / 2, 0.1, 3.5, off_x=0.8)
    assert len(pts) == 1                                         # 3.5 = 끝값, (0.8, 0.3) = 차체 안 -> 뺌
    x, y = pts[0]
    assert abs(x - 0.8) < 1e-9 and abs(y + 2.0) < 1e-9           # -90° 방향 2 m


# ---------------- 노드 모듈 import ----------------
def test_mission_manager_variants_import():
    """mission_manager 에서 이름을 가져가는 변형 노드들이 import 된다 (IDLE 이 빠져 KNU 가 시작부터 죽은 적이 있음)."""
    import importlib

    import pytest
    pytest.importorskip('rclpy')                 # ROS 없이 돌릴 때는 건너뜀
    for name in ('mission_manager', 'mission_manager_campus', 'mission_manager_station', 'extended_runtime'):
        importlib.import_module(f'kickboard_robot.{name}')


# ---------------- collector 정렬 자리 채점 ----------------
RADII = [1.15, 1.30, 1.45, 1.60]


def _spot_grid(scooter_yaw=0.0):
    g = collector_logic.SpotGrid(0.0, 0.0, half=6.0, res=0.1)
    g.add_box(0.0, 0.0, 1.1, 0.3, scooter_yaw, layer='scan')   # 넘어진 킥보드 자신 (라이다에 보임)
    return g


def test_spot_open_space_prefers_robot_side_nominal_radius():
    g = _spot_grid(math.pi / 2)                       # 로봇 쪽에서 보면 가로로 누운 킥보드
    cands, why = collector_logic.plan_align_spots(g, 0.0, 0.0, (-2.6, 0.0), 1.35, RADII)
    best = cands[0]
    assert abs(collector_logic.wrap_angle(best['a'] - math.pi)) <= math.radians(25)   # 로봇 쪽
    assert abs(best['r'] - 1.35) <= 0.16
    # 로봇 쪽으로 길게 누운 킥보드: 기본 반경이면 앞 범퍼가 킥보드 끝에 걸리므로 조금 물러난 자리 (팔은 닿음)
    cands, _ = collector_logic.plan_align_spots(_spot_grid(0.0), 0.0, 0.0, (-2.6, 0.0), 1.35, RADII)
    assert abs(collector_logic.wrap_angle(cands[0]['a'] - math.pi)) <= math.radians(25)
    assert 1.35 < cands[0]['r'] <= 1.80


def test_spot_tree_between_is_avoided():
    g = _spot_grid()
    g.add_box(-0.75, 0.0, 0.8, 0.8, layer='scan')     # 로봇 쪽 킥보드 바로 옆 나무
    cands, why = collector_logic.plan_align_spots(g, 0.0, 0.0, (-2.6, 0.0), 1.35, RADII)
    assert cands, why
    for c in cands:                                   # 어떤 후보도 나무 쪽(±30°)이 아니고, 몸체·팔이 안 걸림
        assert abs(collector_logic.wrap_angle(c['a'] - math.pi)) > math.radians(30)
    assert why['body'] + why['arm'] + why['too_close'] > 0


def test_spot_enclosed_robot_has_no_candidate():
    g = _spot_grid()
    for k in range(4):                                # 로봇 둘레 사방 벽
        a = k * math.pi / 2
        g.add_box(-2.6 + 1.45 * math.cos(a), 1.45 * math.sin(a), 0.3, 3.2, a, layer='scan')
    cands, why = collector_logic.plan_align_spots(g, 0.0, 0.0, (-2.6, 0.0), 1.35, RADII)
    assert cands == [] and why['unreachable'] > 0


def test_spot_sidewalk_keepout_stands_on_road_edge():
    g = _spot_grid()
    g.add_box(0.0, 1.6, 12.0, 4.0, layer='keep')      # y -0.4 ~ 3.6 = 인도 (킥보드는 연석에서 0.4 m 안쪽)
    cands, _ = collector_logic.plan_align_spots(g, 0.0, 0.0, (-1.0, -2.2), 1.35, RADII, keep_clear=0.8)
    assert cands and all(c['y'] <= -0.4 - 0.8 + 0.1 for c in cands)
    assert math.hypot(cands[0]['x'], cands[0]['y']) <= 1.80
    # keep_clear 없음 = 진입 금지 구역을 벽처럼: 몸체가 인도에 걸치는 자리는 빠짐
    cands2, _ = collector_logic.plan_align_spots(g, 0.0, 0.0, (-1.0, -2.2), 1.35, RADII)
    assert all(c['y'] <= -0.4 - 0.47 for c in cands2)


def test_spot_next_skips_failed_spots():
    cands = [{'x': 1.0, 'y': 0.0, 'score': 1}, {'x': 1.3, 'y': 0.0, 'score': 2},
             {'x': 0.94, 'y': 0.34, 'score': 3}, {'x': 0.0, 'y': 1.0, 'score': 4}]
    assert collector_logic.next_spot(cands, [], 0.6)['score'] == 1
    # 실패한 자리 (1, 0) 에서 min_dist 안은 뺀다. 각도는 따지지 않음: 20° 차이(0.34 m)라도 min_dist 를 넘으면 후보
    assert collector_logic.next_spot(cands, [(1.0, 0.0)], 0.32)['score'] == 3
    assert collector_logic.next_spot(cands, [(1.0, 0.0)], 0.6)['score'] == 4
    assert collector_logic.next_spot(cands, [(1.0, 0.0), (0.0, 1.0)], 0.6) is None
    # 팔이 조금 안 닿은 자리: 그 자리만의 작은 거리(0.25)로 빼면 같은 방향 0.3 m 안쪽 자리는 남는다
    assert collector_logic.next_spot(cands, [(1.0, 0.0, 0.25)], 0.6)['score'] == 2


def test_crop_occupancy_reads_map_rows_top_down():
    import numpy as np
    m = np.zeros((40, 40), dtype=bool)                # 4 x 4 m, 0.1 m, 원점 (-2, -2). 위쪽 행 = y 큰 쪽
    m[0:10, :] = True                                 # y 1 ~ 2 m 가 벽
    g = collector_logic.SpotGrid(0.0, 0.0, half=3.0, res=0.1)
    w = collector_logic.crop_occupancy(m, -2.0, -2.0, 0.1, g)
    assert w[g.cell(0.0, 1.5)] and not w[g.cell(0.0, 0.5)] and w[g.cell(0.0, 2.5)]   # 지도 밖 = 장애물


def test_face_tag_ids_and_heading():
    from kickboard_robot import collector_logic as cl
    assert cl.split_tag_id(cl.face_tag_id(7, 2)) == (7, 2)
    assert cl.split_tag_id(100) == (100, 0)          # 충전소 태그
    # 킥보드 (1, 2), 방향 30°: 뒤 면(법선 210°) 검출 -> 중심·방향 복원
    hd = math.radians(30)
    fx, fy = 1 - 0.30 * math.cos(hd), 2 - 0.30 * math.sin(hd)
    cx, cy, h = cl.scooter_from_face(fx, fy, hd + math.pi, 1)
    assert abs(cx - 1) < 1e-9 and abs(cy - 2) < 1e-9 and abs(cl.wrap_angle(h - hd)) < 1e-9


def test_reverse_entry_pose_puts_scooter_behind_lane():
    from kickboard_robot import collector_logic as cl
    for lane in (0, 1):
        x, y, yaw = cl.reverse_entry_pose(0.0, 0.0, math.pi / 2, lane)
        assert abs(yaw - math.pi / 2) < 1e-9
        # 로봇 기준 킥보드 중심 = (-2.07, 레인 y): 뒤 램프 쪽, 그 레인 줄
        c, s = math.cos(yaw), math.sin(yaw)
        rx, ry = c * (0 - x) + s * (0 - y), -s * (0 - x) + c * (0 - y)
        assert abs(rx + cl.ENTRY_DIST) < 1e-9 and abs(ry - cl.LANE_Y[lane]) < 1e-9
        assert all(abs(e) < 1e-9 for e in cl.entry_error((x, y, yaw), 0.0, 0.0, math.pi / 2, lane))


def test_undock_until_body_out_of_station_and_straight():
    st = (10.0, 5.0, math.radians(30))                 # 차고: +x 안쪽
    c, s = math.cos(st[2]), math.sin(st[2])
    pose = [st[0] + 1.3 * c, st[1] + 1.3 * s, st[2] + 0.05]   # 도크 자리, 살짝 틀어짐
    pose[0] -= 0.08 * s
    pose[1] += 0.08 * c                                 # 옆으로 8 cm
    assert ml.body_in_station(pose, st)
    steps = 0
    while ml.body_in_station(pose, st) and steps < 2000:
        v, w = ml.undock_twist(pose, st, 0.2)
        assert v < 0
        pose = [pose[0] + v * math.cos(pose[2]) * 0.05, pose[1] + v * math.sin(pose[2]) * 0.05, pose[2] + w * 0.05]
        steps += 1
    sx, sy = ml.to_station(st, pose[0], pose[1])
    assert sx < -0.45 - 0.2 - 0.96 + 1e-6              # 차체 앞끝까지 램프 밖
    assert abs(sy) < 0.05 and abs(pose[2] - st[2]) < 0.05


def _simulate_dock(start, target, goal_along=0.0, turn_gain=1.0, dt=0.05, steps=3000):
    from kickboard_robot.fake_base import integrate
    x, y, th = start
    for _ in range(steps):
        v, w, done = collector_logic.dock_step((x, y, th), target, goal_along)
        if done:
            return (x, y, th), True
        assert abs(w) <= abs(v) / 1.8 + 1e-9                 # 서서 돌지 않음
        x, y, th = integrate(x, y, th, v, turn_gain * w, dt)
    return (x, y, th), False


def test_dock_reverses_onto_target_from_offset_staging():
    target = collector_logic.reverse_entry_pose(3.0, -1.0, 0.7, 0)
    c, s = math.cos(0.7), math.sin(0.7)
    for lat, head, gain in ((0.25, 8.0, 1.0), (-0.25, -10.0, 0.9), (0.4, 0.0, 1.05)):   # gain: 회전 오차
        start = (target[0] + 3.0 * c - lat * s, target[1] + 3.0 * s + lat * c, 0.7 + math.radians(head))
        pose, done = _simulate_dock(start, target, turn_gain=gain)
        ea, el, ey = collector_logic.dock_errors(pose, target)
        assert done and collector_logic.dock_ok(pose, target) and abs(math.degrees(ey)) < 4, (lat, head, gain, ea, el, ey)
        back, done = _simulate_dock(pose, target, goal_along=3.0, turn_gain=gain)       # 다시 빼기
        assert done and abs(collector_logic.dock_errors(back, target)[1]) < 0.1
