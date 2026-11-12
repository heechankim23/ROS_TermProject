"""가공 시간 측정 (선택 기능, ROS 없음).

켜기:  KICKBOARD_PROFILE=1 ros2 launch kickboard_sim bringup.launch.py ...
       -> 각 노드가 KICKBOARD_PROFILE_SEC(기본 10)초마다 구간별 처리 시간을 로그로 남긴다.
       KICKBOARD_PROFILE_DIR=경로 를 주면 노드가 끝날 때 <경로>/<노드>.json 에 누적 통계를 쓴다.
꺼져 있으면 section() 은 아무 일도 하지 않는 공용 객체를 돌려줘서 비용이 거의 없다.

노드에서 쓰는 법:
    self.prof = Profiler(self.get_name(), self.get_logger().info)
    with self.prof.section('filter_scan'):
        ... 가공 ...
"""
import atexit
import contextlib
import json
import os
import time


def _env_on(name):
    return os.environ.get(name, '').strip().lower() not in ('', '0', 'false', 'no')


_NULL = contextlib.nullcontext()


class _Timer:
    __slots__ = ('prof', 'key', 't0')

    def __init__(self, prof, key):
        self.prof, self.key = prof, key

    def __enter__(self):
        self.t0 = time.perf_counter()

    def __exit__(self, *exc):
        self.prof.record(self.key, (time.perf_counter() - self.t0) * 1000.0)
        return False


class Profiler:
    def __init__(self, name, log=print, enabled=None, report_sec=None, out_dir=None):
        self.name, self.log = name, log
        self.enabled = _env_on('KICKBOARD_PROFILE') if enabled is None else enabled
        self.report_sec = float(os.environ.get('KICKBOARD_PROFILE_SEC', '10')) if report_sec is None else report_sec
        self.out_dir = os.environ.get('KICKBOARD_PROFILE_DIR', '') if out_dir is None else out_dir
        self.window = {}       # key -> [ms, ...] (이번 보고 구간)
        self.total = {}        # key -> [횟수, 합 ms, 최대 ms]
        self.last = time.monotonic()
        if self.enabled and self.out_dir:
            atexit.register(self.dump)

    def section(self, key):
        return _Timer(self, key) if self.enabled else _NULL

    def wrap(self, key, fn):
        """콜백 전체를 잴 때: create_subscription(..., self.prof.wrap('on_scan', self.on_scan), ...)"""
        if not self.enabled:
            return fn

        def timed(*a, **kw):
            with self.section(key):
                return fn(*a, **kw)
        return timed

    def record(self, key, ms):
        self.window.setdefault(key, []).append(ms)
        t = self.total.setdefault(key, [0, 0.0, 0.0])
        t[0] += 1
        t[1] += ms
        t[2] = max(t[2], ms)
        now = time.monotonic()
        if now - self.last >= self.report_sec:
            self.report(now - self.last)
            self.last = now

    def summary(self, span_sec=None):
        """{key: {n, hz, mean_ms, p95_ms, max_ms}} — 이번 구간 기준"""
        out = {}
        for key, v in sorted(self.window.items()):
            s = sorted(v)
            out[key] = {'n': len(s), 'hz': len(s) / span_sec if span_sec else None,
                        'mean_ms': sum(s) / len(s), 'p95_ms': s[min(len(s) - 1, int(0.95 * len(s)))],
                        'max_ms': s[-1]}
        return out

    def report(self, span_sec):
        parts = [f'{k} {v["n"]}회({v["hz"]:.1f}Hz) 평균 {v["mean_ms"]:.2f} p95 {v["p95_ms"]:.2f} 최대 {v["max_ms"]:.2f} ms'
                 for k, v in self.summary(span_sec).items()]
        if parts:
            self.log('[가공 시간] ' + ' | '.join(parts))
        self.window = {}

    def dump(self):
        if not self.total:
            return
        os.makedirs(self.out_dir, exist_ok=True)
        data = {k: {'n': n, 'mean_ms': s / n, 'max_ms': mx} for k, (n, s, mx) in sorted(self.total.items())}
        with open(os.path.join(self.out_dir, f'{self.name}.json'), 'w') as f:
            json.dump(data, f, ensure_ascii=False, indent=1)
