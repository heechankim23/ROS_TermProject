"""Navigation-only regressions. Requires a sourced ROS environment and interfaces."""
import math
from types import SimpleNamespace

import numpy as np
import pytest
import rclpy
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import LaserScan
from rclpy.action import GoalResponse

from kickboard_robot import nav_bridge as nav
from kickboard_robot.planner import Costmap
from kickboard_interfaces.action import GoToPose


@pytest.fixture
def bridge():
    rclpy.init()
    node = None
    try:
        node = nav.NavBridge()
        yield node
    finally:
        if node is not None:
            node.server.destroy()
            node.destroy_node()
        rclpy.shutdown()


def goal():
    g = GoToPose.Goal()
    g.target_pose.header.frame_id = 'map'
    g.target_pose.pose.position.x = 2.0
    g.target_pose.pose.orientation.w = 1.0
    return g


class Handle:
    def __init__(self, canceled=False):
        self.request = goal()
        self.is_cancel_requested = canceled
        self.state = None

    def abort(self):
        self.state = 'aborted'

    def canceled(self):
        self.state = 'canceled'

    def succeed(self):
        self.state = 'succeeded'

    def publish_feedback(self, feedback):
        pass


@pytest.fixture
def clock(monkeypatch):
    c = SimpleNamespace(wall=0.0, sim=0.0)

    def sleep(dt):
        c.wall += dt
        c.sim += dt

    monkeypatch.setattr(nav, 'time', SimpleNamespace(monotonic=lambda: c.wall, sleep=sleep))
    monkeypatch.setattr(nav, 'now_sec', lambda node: c.sim)
    return c


def prepare(bridge, monkeypatch):
    commands = []
    monkeypatch.setattr(bridge.cmd_pub, 'publish', commands.append)
    monkeypatch.setattr(bridge, 'current_pose', lambda: (0.0, 0.0, 0.0))
    monkeypatch.setattr(bridge, 'scan_ready', lambda: True)
    return commands


def test_cancel_stops_before_return(bridge, monkeypatch, clock):
    commands = prepare(bridge, monkeypatch)
    ok, reason = bridge.run_simple(Handle(True), (2.0, 0.0, 0.0), 1)
    assert not ok and reason == 'canceled'
    assert commands[-1].linear.x == commands[-1].angular.z == 0


@pytest.mark.parametrize('missing', ['pose', 'scan'])
def test_sensor_loss_stops_after_moving(bridge, monkeypatch, clock, missing):
    commands = prepare(bridge, monkeypatch)
    if missing == 'pose':
        monkeypatch.setattr(bridge, 'current_pose', lambda: (0.0, 0.0, 0.0) if clock.wall == 0 else None)
    else:
        monkeypatch.setattr(bridge, 'scan_ready', lambda: clock.wall == 0)
    ok, reason = bridge.run_simple(Handle(), (2.0, 0.0, 0.0), 1)
    assert not ok and reason == 'sensor timeout'
    assert commands[0].linear.x > 0
    assert all(t.linear.x == t.angular.z == 0 for t in commands[1:])


def test_cancel_works_while_waiting_for_initial_sensors(bridge, monkeypatch, clock):
    commands = prepare(bridge, monkeypatch)
    monkeypatch.setattr(bridge, 'current_pose', lambda: None)
    h = Handle(True)
    result = bridge._execute_goal(h)
    assert not result.success and h.state == 'canceled'
    assert commands[-1].linear.x == 0


def test_missing_initial_sensor_fails_without_sim_clock(bridge, monkeypatch, clock):
    commands = prepare(bridge, monkeypatch)
    monkeypatch.setattr(bridge, 'current_pose', lambda: None)
    monkeypatch.setattr(nav, 'now_sec', lambda node: 0.0)
    h = Handle()
    result = bridge._execute_goal(h)
    assert not result.success and h.state == 'aborted'
    assert clock.wall >= 5.0
    assert all(t.linear.x == t.angular.z == 0 for t in commands)


def test_busy_goal_is_rejected(bridge):
    assert bridge.accept_goal(goal()) == GoalResponse.ACCEPT
    assert bridge.accept_goal(goal()) == GoalResponse.REJECT
    bridge.goal_lock.release()


def test_pending_nav2_cancel_rejects_new_goal(bridge):
    bridge.nav2_pending = SimpleNamespace(done=lambda: False)
    assert bridge.accept_goal(goal()) == GoalResponse.REJECT


def test_exception_stops_and_releases_goal(bridge, monkeypatch):
    commands = prepare(bridge, monkeypatch)
    assert bridge.accept_goal(goal()) == GoalResponse.ACCEPT

    def fail(handle):
        raise RuntimeError('planner failure')

    monkeypatch.setattr(bridge, '_execute_goal', fail)
    h = Handle()
    assert not bridge._execute(h).success
    assert h.state == 'aborted' and not bridge.goal_lock.locked()
    assert commands[-1].linear.x == commands[-1].angular.z == 0


def test_stale_transform_is_not_a_current_pose(bridge, monkeypatch):
    transform = TransformStamped()
    transform.header.frame_id = 'map'
    transform.child_frame_id = 'base_footprint'
    transform.header.stamp.sec = 10
    transform.transform.rotation.w = 1.0
    bridge.pose.buf.set_transform(transform, 'test')
    monkeypatch.setattr(nav, 'now_sec', lambda node: 10.5)
    assert bridge.current_pose() == (0.0, 0.0, 0.0)
    monkeypatch.setattr(nav, 'now_sec', lambda node: 12.0)
    assert bridge.current_pose() is None


@pytest.mark.parametrize('ranges,ready', [([float('nan')], False), ([], False),
                                        ([float('-inf')], False), ([float('inf')], True),
                                        ([0.12], True), ([0.05], False)])
def test_unknown_forward_scan_is_not_clear(bridge, monkeypatch, clock, ranges, ready):
    monkeypatch.setattr(bridge, 'current_pose', lambda: (0.0, 0.0, 0.0))
    scan = LaserScan()
    scan.range_min, scan.range_max = 0.12, 3.5
    scan.ranges = ranges
    bridge.on_scan(scan)
    assert bridge.scan_ready() == ready
    if ranges == [0.12]:
        assert bridge.front_min < bridge.get_parameter('obstacle_stop_dist').value


def test_scan_reception_timeout(bridge, monkeypatch, clock):
    monkeypatch.setattr(bridge, 'current_pose', lambda: (0.0, 0.0, 0.0))
    scan = LaserScan()
    scan.range_min, scan.range_max = 0.12, 3.5
    scan.ranges = [float('inf')]
    bridge.on_scan(scan)
    assert bridge.scan_ready()
    clock.wall = 1.1
    assert not bridge.scan_ready()


def test_no_safe_standoff_has_no_unchecked_fallback(bridge, monkeypatch):
    prepare(bridge, monkeypatch)
    bridge.cm = object()
    monkeypatch.setattr(nav, 'free_spot', lambda *args: None)
    g = goal()
    g.standoff = 1.6
    assert bridge.make_goal(g) is None


def test_front_obstacle_stops_translation_and_rotation(bridge, monkeypatch, clock):
    commands = prepare(bridge, monkeypatch)
    bridge.front_min = 0.1
    ok, reason = bridge.run_simple(Handle(), (2.0, 0.3, 0.0), 1)
    assert not ok and reason == 'blocked'
    assert all(t.linear.x == t.angular.z == 0 for t in commands)


def test_grid_coordinates_outside_map_do_not_round_inside():
    cm = Costmap(0, 0, 4, 4, res=1, inscribed=0.1, inflate=0.2)
    assert cm.cell(-0.01, 0.5) == (0, -1)
    assert cm.plan((-0.01, 0.5), (2.5, 2.5), 0) is None


def test_astar_cannot_cut_blocked_corner():
    cm = Costmap(0, 0, 3, 3, res=1, inscribed=0.1, inflate=0.2)
    cm.static[0, 1] = cm.static[1, 0] = True
    assert cm.plan((0.5, 0.5), (1.5, 1.5), 0, escape=0) is None


def test_shortcut_cannot_cut_blocked_corner():
    cm = Costmap(0, 0, 3, 3, res=1, inscribed=0.1, inflate=0.2)
    cost = np.ones((3, 3))
    cost[0, 1] = float('inf')
    assert not cm.segment_ok((0.5, 0.5), (1.5, 1.5), cost, 1e18)


def test_unknown_lidar_does_not_clear_remembered_obstacle():
    cm = Costmap(0, 0, 5, 5, res=1, inscribed=0.1, inflate=0.2)
    cm.seen[0, 2] = 0.0
    cm.update_scan(0.5, 0.5, 0.0, [float('nan')], 0, 0.1, 0.1, 4, 1)
    assert cm.seen[0, 2] == 0.0
    cm.update_scan(0.5, 0.5, 0.0, [float('inf')], 0, 0.1, 0.1, 4, 1)
    assert math.isinf(cm.seen[0, 2]) and cm.seen[0, 2] < 0


def test_kinematic_action_arrives_and_stops(bridge):
    """Exercise real ROS Action, TF, scan, cmd_vel and the existing fake base."""
    import time
    from rclpy.node import Node
    from rclpy.action import ActionClient
    from rclpy.executors import SingleThreadedExecutor
    from tf2_ros import StaticTransformBroadcaster
    from kickboard_robot.fake_base import FakeBase

    base = FakeBase()
    driver = Node('navigation_test_driver')
    executor = SingleThreadedExecutor()
    client = ActionClient(driver, GoToPose, '/nav_bridge/go_to_pose')
    scan_pub = driver.create_publisher(LaserScan, '/scan', 10)
    broadcaster = StaticTransformBroadcaster(driver)
    transform = TransformStamped()
    transform.header.stamp = driver.get_clock().now().to_msg()
    transform.header.frame_id, transform.child_frame_id = 'map', 'odom'
    transform.transform.rotation.w = 1.0
    broadcaster.sendTransform(transform)

    def scan():
        msg = LaserScan()
        msg.header.stamp = driver.get_clock().now().to_msg()
        msg.header.frame_id = 'base_scan'
        msg.range_min, msg.range_max = 0.12, 3.5
        msg.ranges = [float('inf')]
        scan_pub.publish(msg)

    driver.create_timer(0.05, scan)
    for node in (bridge, base, driver):
        executor.add_node(node)

    def until(predicate, timeout=12):
        deadline = time.monotonic() + timeout
        while not predicate() and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.02)
        assert predicate()

    try:
        until(lambda: client.server_is_ready() and bridge.scan_ready())
        request = goal()
        request.target_pose.pose.position.x = 0.4
        sent = client.send_goal_async(request)
        until(sent.done)
        assert sent.result().accepted
        result = sent.result().get_result_async()
        until(result.done)
        assert result.result().result.success
        until(lambda: base.v == 0 and base.w == 0)
        assert abs(base.x - 0.4) <= 0.08
    finally:
        executor.shutdown()
        client.destroy()
        driver.destroy_node()
        base.destroy_node()
