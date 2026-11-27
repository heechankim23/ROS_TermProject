#!/usr/bin/env python3
"""빈 world 에서 로봇 모델의 회전 특성 측정 (docs/PLAN_CARLIKE_DRIVING.md 1단계).

  source /opt/ros/humble/setup.bash; export GZ_VERSION=harmonic
  python3 tools/measure_turning.py src/kickboard_sim/models/kickboard_robot/model.sdf 1.0 0.1 1.0 1.0
인자: model.sdf 앞mu 앞mu2 뒤mu 뒤mu2.  환경변수 GY=-0.98 이면 옆경사 10 %. 별도 GZ_PARTITION 을 쓴다."""
import math, os, re, signal, subprocess, sys, tempfile, threading, time
os.environ['GZ_PARTITION'] = f'plan1_{os.getpid()}'
from gz.transport13 import Node
from gz.msgs10.twist_pb2 import Twist
from gz.msgs10.pose_pb2 import Pose

HERE = tempfile.mkdtemp(prefix='turning_')
model, fm, fm2, rm, rm2 = sys.argv[1], *map(float, sys.argv[2:6])
s = open(model).read()
body = s[s.index('<model'):s.index('</model>') + 8]
body = re.sub(r'<sensor .*?</sensor>', '', body, flags=re.S)
body = re.sub(r'<visual name=.*?</visual>', '', body, flags=re.S)
for wheel, mu, mu2 in (('fl', fm, fm2), ('fr', fm, fm2), ('rl', rm, rm2), ('rr', rm, rm2)):
    i = body.index(f'<link name="wheel_{wheel}">'); j = body.index('</link>', i)
    seg = re.sub(r'<mu>[^<]*</mu><mu2>[^<]*</mu2>', f'<mu>{mu}</mu><mu2>{mu2}</mu2>', body[i:j])
    body = body[:i] + seg + body[j:]
body = body.replace('<model name="kickboard_robot">', '<model name="kickboard_robot"><pose>0 0 0.01 0 0 0</pose>')
world = f'''<?xml version="1.0"?><sdf version="1.9" xmlns:gz="http://gazebosim.org/schema"><world name="w"><gravity>0 {float(os.environ.get("GY", 0))} -9.8</gravity>
<physics name="default" type="ignored"><max_step_size>0.01</max_step_size><real_time_factor>0</real_time_factor></physics>
<plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
<plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
<model name="ground"><static>true</static><link name="l"><collision name="c"><geometry><plane><normal>0 0 1</normal><size>400 400</size></plane></geometry></collision></link></model>
{body}</world></sdf>'''
wf = os.path.join(HERE, f'w_{os.getpid()}.sdf'); open(wf, 'w').write(world)
sim = subprocess.Popen(['gz', 'sim', '-s', '-r', wf], stdout=open(os.path.join(HERE, 'sim.log'), 'w'), stderr=subprocess.STDOUT, start_new_session=True)
poses, lock = [], threading.Lock()

def on_pose(m):
    if m.name != 'kickboard_robot':
        return
    t = m.header.stamp.sec + m.header.stamp.nsec * 1e-9
    o = m.orientation
    yaw = math.atan2(2 * (o.w * o.z + o.x * o.y), 1 - 2 * (o.y * o.y + o.z * o.z))
    with lock:
        poses.append((t, m.position.x, m.position.y, yaw))

try:
    node = Node()
    node.subscribe(Pose, '/model/kickboard_robot/pose', on_pose)
    pub = node.advertise('/cmd_vel', Twist)
    t0 = time.time()
    while not poses:
        time.sleep(0.1)
        if time.time() - t0 > 40:
            raise SystemExit('pose 없음')

    def now():
        with lock:
            return poses[-1][0]

    def drive(v, w, sec):
        msg = Twist(); msg.linear.x = v; msg.angular.z = w
        end, wall = now() + sec, time.time()
        while now() < end and time.time() - wall < 120:
            pub.publish(msg); time.sleep(0.01)

    out = []
    for label, v, w in (('제자리', 0.0, 0.5), ('R1.8 저속', 0.5, 0.5 / 1.8), ('R1.8 1.2m/s', 1.2, 1.2 / 1.8),
                        ('R1.8 우회전', 0.8, -0.8 / 1.8), ('R1.8 후진', -0.5, 0.5 / 1.8), ('직진', 0.8, 0.0)):
        drive(0, 0, 1.0); drive(v, w, 3.0)
        with lock:
            n0 = len(poses)
        drive(v, w, 4.0)
        with lock:
            seg = list(poses[n0:])
        vx = vy = wz = T = 0.0
        for a, b in zip(seg, seg[1:]):
            dt = b[0] - a[0]
            if dt <= 0:
                continue
            dyaw = math.atan2(math.sin(b[3] - a[3]), math.cos(b[3] - a[3]))
            th = a[3] + dyaw / 2
            dx, dy = b[1] - a[1], b[2] - a[2]
            vx += math.cos(th) * dx + math.sin(th) * dy
            vy += -math.sin(th) * dx + math.cos(th) * dy
            wz += dyaw; T += dt
        vx, vy, wz = vx / T, vy / T, wz / T
        if w:
            out.append(f'{label}: 회전 {100 * wz / w:4.0f}% 속도 {vx:+.2f} 회전중심x {-vy / wz if abs(wz) > 0.02 else float("nan"):+.2f} '
                       f'뒤축 옆속도 {vy - 0.62 * wz:+.3f}')
        else:
            out.append(f'{label}: 속도 {vx:+.2f} 옆 {vy:+.3f} 회전 {wz:+.3f} rad/s')
    print(f'앞({fm},{fm2}) 뒤({rm},{rm2})\n  ' + '\n  '.join(out))
finally:
    os.killpg(sim.pid, signal.SIGINT)
    try:
        sim.wait(10)
    except subprocess.TimeoutExpired:
        os.killpg(sim.pid, signal.SIGKILL)
    os.remove(wf)
