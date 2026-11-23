#!/usr/bin/env python3
"""시나리오 회귀 테스트: 시나리오를 하나씩 띄우고, 끝나면 통과 기준과 지표를 판정한다.

  source install/setup.bash
  python3 tools/run_scenarios.py                         # 전체, Gazebo 화면 없이
  python3 tools/run_scenarios.py s1_single s6_verify_fail --sim kinematic
  python3 tools/run_scenarios.py s7_obstacle --gui        # Gazebo 창 띄워서 보기

동작:
  - ROS_DOMAIN_ID / GZ_PARTITION 을 따로 잡아서, 같은 PC 에서 돌고 있는 다른 시뮬과 섞이지 않는다.
  - 대기열이 비고 로봇이 IDLE 로 돌아오면(마지막 시나리오 이벤트 + 30 s 이후) 끝난 것으로 본다.
  - 결과: <out>/summary.md (표), <out>/summary.json, 시나리오별 launch 로그·이벤트(jsonl)·rosbag.
  - 하나라도 통과 못 하면 종료 코드 1 (CI 용).
통과 기준은 tools/scenario_checks.py, 지표는 kickboard_robot.analyze 와 같은 계산.
"""
import argparse
import json
import math
import os
import signal
import subprocess
import sys
import time

import rclpy
import yaml
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from std_msgs.msg import String

from kickboard_interfaces.msg import QueueStatus, RobotStatus
from kickboard_robot.analyze import summarize

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import scenario_checks  # noqa: E402

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SCN_DIR = os.path.join(ROOT, 'src', 'kickboard_sim', 'config', 'scenarios')
TOPICS = ['/mission_events', '/robot_status', '/queue_status', '/sim/robot_pose']
TIMEOUT = {'s3_full_load': 1200, 's5_aging': 1500, 's9_drop_sites': 1200}
ACTIVE = {'WAITING', 'IN_PROGRESS'}


def log(msg):
    print(time.strftime('%H:%M:%S'), msg, flush=True)


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class Monitor(Node):
    def __init__(self):
        super().__init__('scenario_monitor')
        self.reset([])
        self.create_subscription(RobotStatus, '/robot_status', self.on_status, 10)
        self.create_subscription(QueueStatus, '/queue_status', self.on_queue, 10)
        self.create_subscription(String, '/mission_events', self.on_event, 100)
        self.create_subscription(PoseStamped, '/sim/robot_pose', self.on_pose, 10)

    def reset(self, boxes):
        self.first, self.state, self.batt, self.load = None, None, None, None
        self.items, self.events = {}, []
        self.boxes, self.min_clear = boxes, float('inf')

    def on_status(self, m):
        if self.first is None:
            self.first = time.time()
        self.state, self.batt, self.load = m.state, m.battery_pct, m.load_count

    def on_queue(self, m):
        self.items = {i.id: i.status for i in m.items}

    def on_event(self, m):
        self.events.append(json.loads(m.data))

    def on_pose(self, m):
        if self.boxes:
            p = m.pose
            self.min_clear = min([self.min_clear] + [scenario_checks.clearance(p.position.x, p.position.y,
                                                                               yaw_of(p.orientation), b)
                                                     for b in self.boxes])


def stop(proc, grace=30):
    """ros2 launch 에만 Ctrl+C 를 보내 노드들이 차례로 정리되게 한다. 남으면 그룹 전체를 종료."""
    if proc.poll() is not None:
        return
    proc.send_signal(signal.SIGINT)
    try:
        proc.wait(grace)
        return
    except subprocess.TimeoutExpired:
        pass
    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGKILL):
        try:
            os.killpg(proc.pid, sig)
            proc.wait(10)
            return
        except (ProcessLookupError, subprocess.TimeoutExpired):
            pass


def run_one(mon, name, args, out):
    tag = name + ('_nav2' if args.nav == 'nav2' else '') + ('_kin' if args.sim == 'kinematic' else '')
    with open(os.path.join(SCN_DIR, name + '.yaml')) as f:
        sc = yaml.safe_load(f)
    last_t = max([float(s.get('spawn_at', 0)) for s in sc.get('scooters', [])]
                 + [float(e.get('t', 0)) for e in sc.get('events', [])] + [0.0])
    timeout = TIMEOUT.get(name, 900) * args.timeout_scale
    cmd = ['ros2', 'launch', 'kickboard_sim', 'bringup.launch.py', f'scenario:={name}', f'sim:={args.sim}',
           f'nav:={args.nav}', f'gui:={"true" if args.gui else "false"}']
    if args.nav == 'nav2':
        cmd.append('start_delay:=15')
    cmd += args.launch_arg

    mon.reset(scenario_checks.obstacle_boxes(sc) if args.sim == 'gz' else [])
    log(f'=== {tag} 시작 (마지막 이벤트 t={last_t:.0f}s, 제한 {timeout:.0f}s)')
    bag_p = None
    if not args.no_bag:
        bag_p = subprocess.Popen(['ros2', 'bag', 'record', '-o', os.path.join(out, 'bags', tag)] + TOPICS,
                                 stdout=open(os.path.join(out, f'{tag}.bag.log'), 'w'), stderr=subprocess.STDOUT,
                                 start_new_session=True)
        time.sleep(2)
    lf = open(os.path.join(out, f'{tag}.launch.log'), 'w')
    launch_p = subprocess.Popen(cmd, stdout=lf, stderr=subprocess.STDOUT, start_new_session=True)
    t0 = time.time()
    result, done_since, last_state = 'timeout', None, None
    while time.time() - t0 < timeout:
        rclpy.spin_once(mon, timeout_sec=0.2)
        if launch_p.poll() is not None:
            result = f'launch 종료({launch_p.returncode})'
            break
        if mon.state != last_state:
            last_state = mon.state
            if args.verbose:
                log(f'  상태 {mon.state}  배터리 {mon.batt and round(mon.batt)}%  적재 {mon.load}  대기열 {mon.items}')
        started = mon.first is not None and time.time() - mon.first > last_t + 30
        idle = mon.state == 'IDLE' and mon.items and not any(s in ACTIVE for s in mon.items.values())
        if started and idle:
            done_since = done_since or time.time()
            if time.time() - done_since > 8:
                result = 'done'
                break
        else:
            done_since = None
    wall = time.time() - t0
    for _ in range(15):                     # 마지막 이벤트 수신
        rclpy.spin_once(mon, timeout_sec=0.2)
    stop(launch_p)
    if bag_p is not None:
        stop(bag_p, grace=15)
    lf.close()

    events = list(mon.events)
    if mon.boxes and math.isfinite(mon.min_clear):
        events.append({'ev': 'runner_clearance', 'min_m': mon.min_clear, 't': events[-1]['t'] if events else 0.0})
    with open(os.path.join(out, f'{tag}.events.jsonl'), 'w') as f:
        f.write('\n'.join(json.dumps(e, ensure_ascii=False) for e in events))
    ok, items = scenario_checks.check(name, events, sc)
    if result != 'done':
        ok = False
        items.insert(0, (False, f'실행: {result}'))
    metrics = summarize([e for e in events if e['ev'] != 'runner_clearance'])
    log(f'=== {tag} {"통과" if ok else "실패"} ({wall:.0f}s)')
    for o, msg in items:
        log(f'    {"o" if o else "x"} {msg}')
    return {'tag': tag, 'scenario': name, 'pass': ok, 'result': result, 'wall_sec': round(wall),
            'checks': [[o, m] for o, m in items], 'metrics': metrics, 'final_items': mon.items}


def write_summary(out, rows, args):
    lines = [f'# 시나리오 회귀 결과 ({time.strftime("%Y-%m-%d %H:%M")}, sim={args.sim}, nav={args.nav})', '',
             '| 시나리오 | 결과 | 시간 | 실패한 기준 |', '|---|---|---|---|']
    for r in rows:
        bad = '; '.join(m for o, m in r['checks'] if not o) or '-'
        lines.append(f'| {r["tag"]} | {"✅ 통과" if r["pass"] else "❌ 실패"} | {r["wall_sec"]} s | {bad} |')
    lines += ['', '## 지표', '']
    for r in rows:
        lines += [f'### {r["tag"]}', '```', r['metrics'], '```', '']
    with open(os.path.join(out, 'summary.md'), 'w') as f:
        f.write('\n'.join(lines))
    with open(os.path.join(out, 'summary.json'), 'w') as f:
        json.dump(rows, f, ensure_ascii=False, indent=1)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('scenarios', nargs='*', help='시나리오 이름 (기본: config/scenarios 전체)')
    ap.add_argument('--sim', default='gz', choices=['gz', 'kinematic'])
    ap.add_argument('--nav', default='simple', choices=['simple', 'nav2'])
    ap.add_argument('--gui', action='store_true', help='Gazebo 창 띄우기')
    ap.add_argument('--out', default=None, help='결과 폴더 (기본: runs/<시각>)')
    ap.add_argument('--domain', type=int, default=int(os.environ.get('KICKBOARD_TEST_DOMAIN', '42')),
                    help='ROS_DOMAIN_ID (다른 시뮬과 분리)')
    ap.add_argument('--timeout-scale', type=float, default=1.0, help='시간 제한 배율 (느린 PC)')
    ap.add_argument('--no-bag', action='store_true', help='rosbag 기록 안 함')
    ap.add_argument('--launch-arg', action='append', default=[], help='bringup 에 넘길 인자 (예: arm:=false)')
    ap.add_argument('-v', '--verbose', action='store_true', help='상태 변화마다 출력')
    args = ap.parse_args()

    names = args.scenarios or sorted(f[:-5] for f in os.listdir(SCN_DIR) if f.endswith('.yaml'))
    out = args.out or os.path.join(ROOT, 'runs', time.strftime('%Y%m%d_%H%M%S'))
    os.makedirs(os.path.join(out, 'bags'), exist_ok=True)
    os.environ['ROS_DOMAIN_ID'] = str(args.domain)
    os.environ.setdefault('GZ_PARTITION', f'kickboard_test_{args.domain}')
    rclpy.init()
    mon = Monitor()
    rows = []
    try:
        for n in names:
            rows.append(run_one(mon, n, args, out))
            write_summary(out, rows, args)
            time.sleep(5)
    finally:
        mon.destroy_node()
        rclpy.shutdown()
    n_ok = sum(r['pass'] for r in rows)
    log(f'전체 {n_ok}/{len(rows)} 통과 -> {os.path.join(out, "summary.md")}')
    sys.exit(0 if n_ok == len(rows) else 1)


if __name__ == '__main__':
    main()
