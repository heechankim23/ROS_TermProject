#!/usr/bin/env python3
"""경북대 캠퍼스 시나리오(k00~k32)를 차례로(또는 동시에 여러 개) 돌리고, 로봇이 일을 마치면 끄고 다음으로 넘어간다.
끝나면 시나리오별 통과 여부(check_knu_runs.py)와 운영 리플레이(build_ops_replay.py)를 만든다.

  python3 scripts/run_knu_scenarios.py                    # 전부, 시나리오 전용 작은 world, Gazebo 화면 없이
  python3 scripts/run_knu_scenarios.py --gui --rtf 4      # 화면을 띄우고 4배속
  python3 scripts/run_knu_scenarios.py --full             # 캠퍼스 전체 world 로 (느림)
  python3 scripts/run_knu_scenarios.py k01 k08 k13        # 고른 것만
  python3 scripts/run_knu_scenarios.py --gui              # Gazebo 화면을 띄워서 (하나씩)
  python3 scripts/run_knu_scenarios.py --kinematic -j 4   # Gazebo 없이 로직만, 4개 동시에 (ROS_DOMAIN_ID 를 나눠 씀)
  python3 scripts/run_knu_scenarios.py --speed 3.0        # 로봇 최고 속도(m/s) 덮어쓰기 (기본 1.2 = 모터 한계, 그 이상은 Gazebo 에서 잘림)

'끝' 판정: 로봇이 대기(IDLE)로 돌아온 뒤 --grace 초 동안 새 일이 없고, 시나리오의 마지막 이벤트·등장 시각도 지났을 때.
시나리오당 최대 --limit 초. 로그: my_ws/results/knu_<날짜_시각>/<시나리오>.log
"""
import argparse
import glob
import os
import re
import signal
import subprocess
import sys
import time

import yaml

PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
WS = os.path.dirname(os.path.dirname(PKG))
SC_DIR = os.path.join(PKG, 'config', 'scenarios_knu')
ANSI = re.compile(r'\x1b\[[0-9;]*m')


def last_event_time(sc):
    t = [float(e.get('t', 0)) for e in sc.get('events') or []]
    t += [float(s.get('spawn_at', 0)) for s in sc.get('scooters') or []]
    return max(t or [0.0])


class Run:
    def __init__(self, name, args, out, domain):
        self.name, self.args, self.out, self.domain = name, args, out, domain
        src = os.path.join(SC_DIR, name + '.yaml')
        sc = yaml.safe_load(open(src))
        if args.speed:                                 # 속도 덮어쓰기: 시나리오 사본에 params 를 붙인다
            sc.setdefault('params', {}).setdefault('nav_bridge', {}).update(
                {'v_max': float(args.speed), 'w_max': min(2.0, 0.6 + 0.3 * float(args.speed))})
        self.scen = os.path.join(out, name + '.yaml')
        yaml.safe_dump(sc, open(self.scen, 'w'), allow_unicode=True, sort_keys=False)
        self.last_ev = last_event_time(sc)
        self.log = os.path.join(out, name + '.log')
        cmd = ['ros2', 'launch', 'kickboard_sim', 'bringup.launch.py', f'scenario:={self.scen}']
        if args.kinematic:
            cmd.append('sim:=kinematic')
        else:
            cmd += ['start_delay:=15'] + ([] if args.gui else ['gui:=false', 'sensors:=false'])
            if args.rviz:
                cmd.append('rviz:=true')
        if args.map:
            cmd.append(f'map:={args.map}')
        elif not args.full:
            cmd.append('map:=local')                   # 시나리오 전용 작은 캠퍼스 (knu_<시나리오>)
        if not args.kinematic and args.rtf != 1.0:
            cmd.append(f'rtf:={args.rtf}')
        env = dict(os.environ, ROS_DOMAIN_ID=str(domain), GZ_PARTITION=f'knu{domain}')
        self.f = open(self.log, 'w')
        self.p = subprocess.Popen(cmd, stdout=self.f, stderr=subprocess.STDOUT, env=env, start_new_session=True)
        self.t0 = time.monotonic()
        self.idle_since = None
        self.started = None              # 시나리오 시각 0 (첫 '새 요청')
        self.result = None

    def lines(self):
        try:
            return ANSI.sub('', open(self.log, errors='ignore').read()).splitlines()
        except OSError:
            return []

    def poll(self):
        """끝났으면 결과 문자열, 아니면 None"""
        now = time.monotonic()
        if self.p.poll() is not None:
            return self.finish('프로세스 종료')
        if now - self.t0 > self.args.limit:
            return self.finish('시간 초과')
        st = [ln for ln in self.lines() if '[상태]' in ln]
        if self.started is None and any('IDLE -> SELECTING' in ln or 'SELECTING -> NAVIGATING' in ln for ln in st):
            self.started = now
        last = st[-1] if st else ''
        idle = '-> IDLE' in last
        if not idle:
            self.idle_since = None
            return None
        if self.idle_since is None:
            self.idle_since = now
        enough = self.started is not None and now - self.started > self.last_ev * self.time_scale() + 5
        if enough and now - self.idle_since > self.args.grace * self.time_scale():
            return self.finish('완료')
        return None

    def time_scale(self):
        """시나리오 시각 1초가 실제 몇 초인지 (Gazebo 배속이면 1/rtf)"""
        return 1.0 if self.args.kinematic or self.args.rtf <= 0 else 1.0 / self.args.rtf

    def finish(self, why):
        if self.p.poll() is None:
            try:
                os.killpg(self.p.pid, signal.SIGINT)
                self.p.wait(timeout=25)
            except (subprocess.TimeoutExpired, ProcessLookupError):
                try:
                    os.killpg(self.p.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
        self.f.close()
        if not self.args.kinematic:
            subprocess.run(['pkill', '-KILL', '-f', f'gz sim.*{os.path.basename(self.scen)}'], check=False)
        self.result = f'{why} ({(time.monotonic() - self.t0) / 60:.1f}분)'
        return self.result


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('names', nargs='*', help='시나리오 이름 (비우면 k00~k32 전부)')
    ap.add_argument('--gui', action='store_true', help='Gazebo 화면을 띄움 (한 번에 하나)')
    ap.add_argument('--rviz', action='store_true')
    ap.add_argument('--kinematic', action='store_true', help='Gazebo 없이 로직만')
    ap.add_argument('-j', '--jobs', type=int, default=1, help='동시에 돌릴 수 (Gazebo 는 1 권장)')
    ap.add_argument('--speed', type=float, default=0.0, help='로봇 최고 속도 m/s (0 = 맵 기본 1.2)')
    ap.add_argument('--map', default='', help='맵 덮어쓰기 (예: knu_campus = 언덕 지형 전체 캠퍼스)')
    ap.add_argument('--full', action='store_true', help='시나리오 전용 작은 world 대신 캠퍼스 전체 world')
    ap.add_argument('--rtf', type=float, default=1.0, help='Gazebo 배속 (예: 4). kinematic 에는 해당 없음 (--speed 사용)')
    ap.add_argument('--limit', type=float, default=3600, help='시나리오당 최대 초')
    ap.add_argument('--grace', type=float, default=200, help='대기(IDLE) 뒤 이만큼 새 일이 없으면 끝')
    a = ap.parse_args()
    names = a.names or sorted(os.path.basename(p)[:-5] for p in glob.glob(os.path.join(SC_DIR, 'k*.yaml')))
    if a.gui:
        a.jobs = 1
    out = os.path.join(WS, 'results', time.strftime('knu_%m%d_%H%M'))
    os.makedirs(out, exist_ok=True)
    print(f'시나리오 {len(names)}개, 동시 {a.jobs}개, {"kinematic" if a.kinematic else "Gazebo"}'
          f'{" (화면)" if a.gui else ""}{f", {a.rtf}배속" if a.rtf != 1 and not a.kinematic else ""}'
          f'{f", 로봇 속도 {a.speed} m/s" if a.speed else ""}, {"캠퍼스 전체" if a.full else "시나리오 전용 world"} -> {out}', flush=True)
    todo, running, done = list(names), [], []
    t_start = time.monotonic()
    try:
        while todo or running:
            while todo and len(running) < a.jobs:
                name = todo.pop(0)
                dom = int(os.environ.get('KNU_DOMAIN_BASE', 40)) + (len(done) + len(running)) % 50
                running.append(Run(name, a, out, dom))
                print(f'[{time.strftime("%H:%M:%S")}] 시작 {name}', flush=True)
            time.sleep(5)
            for r in list(running):
                res = r.poll()
                if res:
                    running.remove(r)
                    done.append(r)
                    print(f'[{time.strftime("%H:%M:%S")}] 끝 {r.name}: {res}', flush=True)
    except KeyboardInterrupt:
        print('\n중단 — 실행 중인 시나리오를 끕니다', flush=True)
        for r in running:
            r.finish('중단')
            done.append(r)
    print(f'전체 {(time.monotonic() - t_start) / 60:.0f}분', flush=True)
    py = sys.executable
    subprocess.run([py, os.path.join(PKG, 'scripts', 'check_knu_runs.py'), '-o', os.path.join(out, 'summary.txt')], check=False)
    note = 'Gazebo 없는 모드(kinematic)' if a.kinematic else ('Gazebo' + (f' {a.rtf}배속' if a.rtf != 1 else ''))
    note += ', 캠퍼스 전체 world' if a.full else ', 시나리오 전용 world'
    if a.speed:
        note += f', 로봇 속도 {a.speed} m/s (맵 기본 1.5 m/s)'
    subprocess.run([py, os.path.join(PKG, 'scripts', 'build_ops_replay.py'), '--note', note + ' 실행 기록'], check=False)


if __name__ == '__main__':
    main()
