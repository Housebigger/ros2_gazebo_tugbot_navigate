"""Deterministic mock-node tests for ReactivePilot.

A fake clock / node / Nav2 action-client harness drives the pilot's state
machine without a live ROS runtime (only message types are imported). Each
test advances the fake clock and robot pose between tick()s to simulate motion
and asserts the terminal `result` / `is_active()` / queue state.

Covers: Nav2 happy path, the three Nav2->reactive fallback paths, multi-hop
follow_path, mid-path WALL_AHEAD, the no-progress / no-rotation / watchdog
WEDGED paths, back_out (success + rear-blocked), the jam-unwedge contrast,
the I2 stale-result-callback guard, and the I3 None-pose guard.
"""
import math

from sensor_msgs.msg import LaserScan

from tugbot_maze.maze_perception import normalize_angle
from tugbot_maze.reactive_pilot import ReactivePilot, SUCCESS, WALL_AHEAD, WEDGED


# --------------------------------------------------------------------- fakes
class FakeTime:
    """Stands in for rclpy Time and Duration: both expose `.nanoseconds`."""

    def __init__(self, nanoseconds):
        self.nanoseconds = nanoseconds

    def __sub__(self, other):
        return FakeTime(self.nanoseconds - other.nanoseconds)


class FakeClock:
    def __init__(self):
        self._ns = 0

    def now(self):
        return FakeTime(self._ns)

    def advance(self, secs):
        self._ns += int(secs * 1e9)


class FakeLogger:
    def info(self, *a, **k):
        pass

    def warn(self, *a, **k):
        pass

    def error(self, *a, **k):
        pass


class FakeCmdVelPub:
    def __init__(self):
        self.published = []

    def publish(self, twist):
        self.published.append(twist)


class FakeNode:
    def __init__(self, clock, pose=(0.0, 0.0, 0.0)):
        self._clock = clock
        self._logger = FakeLogger()
        self.map_frame = 'map'
        self.scan_msg = None
        self.cmd_vel_pub = FakeCmdVelPub()
        self._pose = pose

    def get_clock(self):
        return self._clock

    def get_logger(self):
        return self._logger

    def _lookup_pose(self):
        return self._pose

    def set_pose(self, x, y, yaw):
        self._pose = (x, y, yaw)


class FakeResult:
    def __init__(self, status):
        self.status = status


class FakeFuture:
    """add_done_callback fires synchronously unless deferred; fire() releases
    any deferred callbacks (used to inject a stale Nav2 result in case 9)."""

    def __init__(self, value, defer=False):
        self._value = value
        self._defer = defer
        self._cbs = []

    def result(self):
        return self._value

    def add_done_callback(self, cb):
        if self._defer:
            self._cbs.append(cb)
        else:
            cb(self)

    def fire(self):
        cbs, self._cbs = self._cbs, []
        for cb in cbs:
            cb(self)


class FakeGoalHandle:
    def __init__(self, accepted, result_future):
        self.accepted = accepted
        self._result_future = result_future
        self.cancel_called = False

    def get_result_async(self):
        return self._result_future

    def cancel_goal_async(self):
        self.cancel_called = True
        return FakeFuture(None)


class FakeActionClient:
    def __init__(self, ready=True, accepted=True, status=4, defer_result=False):
        self.ready = ready
        self.accepted = accepted
        self.status = status
        self.defer_result = defer_result
        self.sent_goals = []
        self.result_futures = []
        self.goal_handles = []

    def server_is_ready(self):
        return self.ready

    def wait_for_server(self, timeout_sec=None):
        return self.ready

    def send_goal_async(self, goal):
        self.sent_goals.append(goal)
        result_future = FakeFuture(FakeResult(self.status), defer=self.defer_result)
        gh = FakeGoalHandle(self.accepted, result_future)
        self.result_futures.append(result_future)
        self.goal_handles.append(gh)
        # The goal-response future resolves immediately to the goal handle.
        return FakeFuture(gh, defer=False)


# ----------------------------------------------------------------- helpers
def _scan(fill, n=72):
    scan = LaserScan()
    scan.angle_min = -math.pi
    scan.angle_increment = 2.0 * math.pi / n
    scan.range_max = 100.0
    scan.ranges = [float(fill)] * n
    return scan


def clear_scan():
    return _scan(50.0)      # every cone clear


def blocked_scan():
    return _scan(0.2)       # every cone blocked (< both min ranges)


def complete_reactive_hop(pilot, node, origin, target):
    """Drive an active 'rotating' reactive hop to its target with a clear scan."""
    assert pilot.state == 'rotating'
    node.scan_msg = clear_scan()
    ox, oy = origin
    node.set_pose(ox, oy, pilot.target_yaw)   # snap heading -> rotation completes
    pilot.tick(node._pose)
    assert pilot.state == 'driving'
    pilot.tick(node._pose)                     # first forward tick (sets progress ref)
    # Move a little past the target so distance_traveled >= target_distance.
    px = ox + 1.10 * (target[0] - ox)
    py = oy + 1.10 * (target[1] - oy)
    node.set_pose(px, py, pilot.target_yaw)
    pilot.tick(node._pose)                      # distance reached -> hop done


def assert_aimed_at(pilot, origin, target):
    expected = math.atan2(target[1] - origin[1], target[0] - origin[0])
    assert abs(normalize_angle(pilot.target_yaw - expected)) < 1e-9


# --------------------------------------------------------------------- tests
def test_case1_nav2_happy_path():
    clock = FakeClock()
    node = FakeNode(clock, pose=(0.0, 0.0, 0.0))
    client = FakeActionClient(ready=True, accepted=True, status=4)
    pilot = ReactivePilot(node, client)

    pilot.drive_to((2.0, 0.0))
    assert pilot.state == 'nav' and pilot.is_active() and pilot.result is None
    assert len(client.sent_goals) == 1

    pilot.tick(node._pose)        # nav_done & succeeded -> hop reached
    assert pilot.result == SUCCESS
    assert not pilot.is_active()
    assert pilot.queue == []


def test_case2_fallback_paths_reach_target_via_reactive():
    target = (1.0, 1.0)

    # (a) server not ready -> fallback happens inside drive_to (no tick).
    clock = FakeClock()
    node = FakeNode(clock, pose=(0.0, 0.0, 0.0))
    client = FakeActionClient(ready=False)
    pilot = ReactivePilot(node, client)
    pilot.drive_to(target)
    assert pilot.state == 'rotating'
    assert_aimed_at(pilot, (0.0, 0.0), target)
    complete_reactive_hop(pilot, node, (0.0, 0.0), target)
    assert pilot.result == SUCCESS and not pilot.is_active()

    # (b) goal rejected -> fallback on first tick.
    clock = FakeClock()
    node = FakeNode(clock, pose=(0.0, 0.0, 0.0))
    client = FakeActionClient(ready=True, accepted=False)
    pilot = ReactivePilot(node, client)
    pilot.drive_to(target)
    assert pilot.state == 'nav'
    pilot.tick(node._pose)        # not accepted -> reactive fallback
    assert pilot.state == 'rotating'
    assert_aimed_at(pilot, (0.0, 0.0), target)
    complete_reactive_hop(pilot, node, (0.0, 0.0), target)
    assert pilot.result == SUCCESS and not pilot.is_active()

    # (c) result status != 4 -> fallback on first tick.
    clock = FakeClock()
    node = FakeNode(clock, pose=(0.0, 0.0, 0.0))
    client = FakeActionClient(ready=True, accepted=True, status=6)
    pilot = ReactivePilot(node, client)
    pilot.drive_to(target)
    assert pilot.state == 'nav'
    pilot.tick(node._pose)        # nav failed -> reactive fallback
    assert pilot.state == 'rotating'
    assert_aimed_at(pilot, (0.0, 0.0), target)
    complete_reactive_hop(pilot, node, (0.0, 0.0), target)
    assert pilot.result == SUCCESS and not pilot.is_active()


def test_case3_follow_path_multi_hop():
    clock = FakeClock()
    node = FakeNode(clock, pose=(0.0, 0.0, 0.0))
    client = FakeActionClient(ready=True, accepted=True, status=4)
    pilot = ReactivePilot(node, client)

    pilot.follow_path([(1.0, 0.0), (2.0, 0.0)])
    assert pilot.state == 'nav'
    assert len(pilot.queue) == 2 and pilot.result is None and pilot.is_active()

    # Hop 1 completes via Nav2; hop 2 must fall back to reactive.
    client.ready = False
    node.set_pose(1.0, 0.0, 0.0)
    pilot.tick(node._pose)        # hop1 reached -> pop -> dispatch hop2 -> reactive
    assert pilot.queue == [(2.0, 0.0)]            # intermediate pop happened
    assert pilot.is_active() and pilot.result is None  # SUCCESS not yet
    assert pilot.state == 'rotating'

    complete_reactive_hop(pilot, node, (1.0, 0.0), (2.0, 0.0))
    assert pilot.result == SUCCESS                # only after the LAST waypoint
    assert not pilot.is_active() and pilot.queue == []


def test_case4_midpath_wall_ahead_halts_path():
    clock = FakeClock()
    node = FakeNode(clock, pose=(0.0, 0.0, 0.0))
    # Nav2 fails (status 3) so hop1 runs reactively where WALL_AHEAD can fire.
    client = FakeActionClient(ready=True, accepted=True, status=3)
    pilot = ReactivePilot(node, client)

    pilot.follow_path([(1.0, 0.0), (2.0, 0.0)])
    assert pilot.state == 'nav' and len(client.sent_goals) == 1

    node.scan_msg = clear_scan()
    pilot.tick(node._pose)        # nav failed -> reactive fallback
    assert pilot.state == 'rotating'
    pilot.tick(node._pose)        # rotation complete -> driving
    assert pilot.state == 'driving'
    pilot.tick(node._pose)        # forward, progress ref @ (0,0)

    node.set_pose(0.5, 0.0, 0.0)  # made some progress
    node.scan_msg = blocked_scan()
    pilot.tick(node._pose)        # wall in forward cone -> halt

    assert pilot.result == WALL_AHEAD
    assert not pilot.is_active()
    assert pilot.queue == []
    assert len(client.sent_goals) == 1    # remaining (2,0) never attempted


def test_case5_no_progress_wedge():
    clock = FakeClock()
    node = FakeNode(clock, pose=(0.0, 0.0, 0.0))
    pilot = ReactivePilot(node, FakeActionClient())

    pilot.reactive_drive(0.0, 2.0)            # forced reactive, heading already 0
    node.scan_msg = clear_scan()
    pilot.tick(node._pose)                    # rotation complete -> driving
    assert pilot.state == 'driving'
    pilot.tick(node._pose)                    # forward, progress ref set

    clock.advance(4.0)                        # > no_progress_sec, pose unchanged
    pilot.tick(node._pose)                    # forward clear but no progress -> backup
    assert pilot.state == 'backup'

    pilot.tick(node._pose)                    # backup ref @ (0,0)
    node.set_pose(-0.5, 0.0, 0.0)             # reversed past _UNWEDGE_BACKUP_M
    pilot.tick(node._pose)
    assert pilot.result == WEDGED and not pilot.is_active()


def test_case6_no_rotation_jam():
    clock = FakeClock()
    node = FakeNode(clock, pose=(0.0, 0.0, 0.0))
    pilot = ReactivePilot(node, FakeActionClient())

    pilot.reactive_drive(math.pi / 2, 1.0)    # needs to rotate 90 deg
    node.scan_msg = clear_scan()
    pilot.tick(node._pose)                    # rotating; rot ref set
    assert pilot.state == 'rotating'

    clock.advance(4.0)                        # > no_rot_sec, yaw unchanged
    pilot.tick(node._pose)                    # cannot rotate -> backup
    assert pilot.state == 'backup'

    pilot.tick(node._pose)                    # backup ref @ (0,0)
    node.set_pose(-0.5, 0.0, 0.0)
    pilot.tick(node._pose)
    assert pilot.result == WEDGED and not pilot.is_active()


def test_case7_watchdog_timeout():
    clock = FakeClock()
    node = FakeNode(clock, pose=(0.0, 0.0, 0.0))
    pilot = ReactivePilot(node, FakeActionClient())   # max_seconds default 12

    pilot.reactive_drive(0.0, 1.0)
    node.scan_msg = clear_scan()
    clock.advance(13.0)                        # past max_seconds
    pilot.tick(node._pose)                     # watchdog fires before state logic
    assert pilot.result == WEDGED and not pilot.is_active()


def test_case8_back_out_success_unwedge_contrast_and_rear_blocked():
    # (a) intentional back_out reverses ~target_distance -> SUCCESS.
    clock = FakeClock()
    node = FakeNode(clock, pose=(0.0, 0.0, 0.0))
    pilot = ReactivePilot(node, FakeActionClient())
    node.scan_msg = clear_scan()
    pilot.back_out(1.0, (0.0, 0.0, 0.0))
    assert pilot.state == 'backup' and pilot._reverse_is_backout is True
    pilot.tick(node._pose)                     # backup ref @ (0,0)
    node.set_pose(-1.1, 0.0, 0.0)              # reversed past target_distance
    pilot.tick(node._pose)
    assert pilot.result == SUCCESS and not pilot.is_active()

    # (b) contrast: a jam-triggered unwedge backs the same way but ends WEDGED.
    clock = FakeClock()
    node = FakeNode(clock, pose=(0.0, 0.0, 0.0))
    pilot = ReactivePilot(node, FakeActionClient())
    node.scan_msg = clear_scan()
    pilot.reactive_drive(0.0, 2.0)
    pilot.tick(node._pose)                     # -> driving
    pilot.tick(node._pose)                     # forward, progress ref
    clock.advance(4.0)
    pilot.tick(node._pose)                     # no progress -> backup (unwedge)
    assert pilot.state == 'backup' and pilot._reverse_is_backout is False
    pilot.tick(node._pose)                     # backup ref @ (0,0)
    node.set_pose(-0.5, 0.0, 0.0)
    pilot.tick(node._pose)
    assert pilot.result == WEDGED and not pilot.is_active()

    # (c) rear cone blocked during reverse -> WEDGED.
    clock = FakeClock()
    node = FakeNode(clock, pose=(0.0, 0.0, 0.0))
    pilot = ReactivePilot(node, FakeActionClient())
    node.scan_msg = blocked_scan()             # wall close behind
    pilot.back_out(1.0, (0.0, 0.0, 0.0))
    pilot.tick(node._pose)                     # rear not clear -> stop
    assert pilot.result == WEDGED and not pilot.is_active()


def test_case9_stale_result_callback_is_ignored():
    clock = FakeClock()
    node = FakeNode(clock, pose=(0.0, 0.0, 0.0))
    # Defer Nav2 results so we can fire an OLD callback after a new dispatch.
    client = FakeActionClient(ready=True, accepted=True, status=4, defer_result=True)
    pilot = ReactivePilot(node, client)

    # Hop 1: dispatched, result still pending (deferred).
    pilot.drive_to((1.0, 0.0))
    assert pilot.state == 'nav' and pilot.nav_done is False
    assert pilot._nav_seq == 1

    # Deadline cancel -> falls back to reactive (no new nav seq yet).
    clock.advance(20.0)                        # past nav_timeout_sec (18)
    pilot.tick(node._pose)
    assert client.goal_handles[0].cancel_called is True

    # A brand-new dispatch supersedes hop 1 (seq advances, result pending).
    pilot.drive_to((2.0, 0.0))
    assert pilot.state == 'nav' and pilot._nav_seq == 2
    assert pilot.nav_done is False and pilot.nav_succeeded is False

    # Firing the OLD (seq 1) result must NOT flip the new hop's flags.
    client.result_futures[0].fire()
    assert pilot.nav_done is False
    assert pilot.nav_succeeded is False

    # The CURRENT (seq 2) result still gets through.
    client.result_futures[1].fire()
    assert pilot.nav_done is True
    assert pilot.nav_succeeded is True


def test_i3_none_pose_guard_does_not_crash():
    clock = FakeClock()
    node = FakeNode(clock, pose=(0.0, 0.0, 0.0))
    pilot = ReactivePilot(node, FakeActionClient())
    pilot.reactive_drive(0.0, 1.0)             # state='rotating'
    pilot.tick(None)                           # must be a no-op, not a crash
    assert pilot.result is None
    assert pilot.is_active()
