"""PRD 7장 측정 지표 계산: rosbag2 의 /mission_events 를 읽어 요약한다.

  ros2 bag record -o bags/s3 /mission_events /robot_status /queue_status
  ros2 run kickboard_robot analyze bags/s3
"""
import json
import math
import sys


def load_events(bag_path):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from std_msgs.msg import String
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=bag_path, storage_id='sqlite3'),
                rosbag2_py.ConverterOptions('cdr', 'cdr'))
    reader.set_filter(rosbag2_py.StorageFilter(topics=['/mission_events']))
    out = []
    while reader.has_next():
        _, data, _ = reader.read_next()
        out.append(json.loads(deserialize_message(data, String).data))
    return out


def summarize(events, w=None):
    truth, gps_err, cor_err = {}, [], []
    real_requests, collected, waits, docks, fails, cancelled = set(), set(), [], [], [], set()
    for e in events:
        ev = e['ev']
        if ev == 'request_sent':
            truth[e['id']] = (e['true_x'], e['true_y'])
            if e.get('has_model', True):
                real_requests.add(e['id'])
            gps_err.append(math.hypot(e['gps_x'] - e['true_x'], e['gps_y'] - e['true_y']))
        elif ev == 'collected':
            collected.add(e['id'])
            if e['id'] in truth:
                tx, ty = truth[e['id']]
                cor_err.append(math.hypot(e['corrected_x'] - tx, e['corrected_y'] - ty))
        elif ev == 'item_status' and e['status'] == 'DONE':
            waits.append((e['wait_sec'], e['id']))
        elif ev == 'item_status' and e['status'] == 'CANCELLED':
            cancelled.add(e['id'])          # 정상으로 돌아와 취소된 요청은 분모에서 제외
        elif ev == 'docked':
            docks.append((e['position_error_m'], e['yaw_error_deg']))
        elif ev == 'collect_failed':
            fails.append((e['id'], e['reason']))

    def avg(v):
        return sum(v) / len(v) if v else float('nan')

    lines = ['=== 측정 지표 ===']
    real_requests -= cancelled - collected
    n_real = len(real_requests)
    ok = len(collected & real_requests)
    lines.append(f'수거 성공률      : {ok}/{n_real} = {100.0 * ok / n_real if n_real else float("nan"):.0f}%  (목표 ≥ 90%)')
    if waits:
        mw = max(waits)
        lines.append(f'최대 대기 시간    : {mw[0]:.1f}s (킥보드 {mw[1]}), 평균 {avg([x[0] for x in waits]):.1f}s')
    if docks:
        lines.append(f'도킹 오차(최대)  : 위치 {max(d[0] for d in docks) * 100:.1f} cm, '
                     f'방향 {max(abs(d[1]) for d in docks):.1f}°  (목표 ≤ 5 cm, ≤ 5°)')
    lines.append(f'위치 오차 GPS    : 평균 {avg(gps_err):.2f} m')
    lines.append(f'위치 오차 보정 후 : 평균 {avg(cor_err):.2f} m, 최대 {max(cor_err, default=float("nan")):.2f} m'
                 f'  (목표 ≤ 0.2 m)')
    if cancelled:
        lines.append(f'취소(제외)       : {sorted(cancelled)}')
    if fails:
        lines.append(f'실패            : {fails}')
    return '\n'.join(lines)


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)
    print(summarize(load_events(sys.argv[1])))


if __name__ == '__main__':
    main()
