import math

try:  # pragma: no cover
    import rclpy
    from geometry_msgs.msg import Twist
    from rclpy.node import Node
    from std_msgs.msg import Float32
except ImportError:  # pragma: no cover
    rclpy = None
    Twist = None
    Float32 = None

    class Node:  # type: ignore[override]
        pass


class SearchState:
    def __init__(self, mode: str = 'idle', progress: float = 0.0, arc_direction: float = 1.0) -> None:
        self.mode = mode
        self.progress = float(progress)
        self.arc_direction = float(arc_direction)

def _sign(value: float, fallback: float = 1.0) -> float:
    if value > 0:
        return 1.0
    if value < 0:
        return -1.0
    return 1.0 if fallback >= 0 else -1.0


def _default_search_state(config: dict, mode: str = 'idle') -> SearchState:
    return SearchState(
        mode=mode,
        progress=0.0,
        arc_direction=_sign(float(config.get('search_default_turn_direction', 1.0))),
    )


def compute_control_command(error, prev_error: float, integral: float, dt: float, config: dict):
    if error is None:
        return 0.0, 0.0, 0.0

    kp = float(config['kp'])
    ki = float(config['ki'])
    kd = float(config['kd'])
    integral_limit = float(config['integral_limit'])
    base_linear_speed = float(config['base_linear_speed'])
    min_linear_speed = float(config['min_linear_speed'])
    max_angular_speed = float(config['max_angular_speed'])
    speed_reduction_gain = float(config['speed_reduction_gain'])

    integral = max(-integral_limit, min(integral_limit, integral + float(error) * dt))
    derivative = (float(error) - prev_error) / dt if dt > 0 else 0.0
    angular_z = kp * float(error) + ki * integral + kd * derivative
    angular_z = max(-max_angular_speed, min(max_angular_speed, angular_z))

    linear_x = max(min_linear_speed, base_linear_speed * (1.0 - speed_reduction_gain * min(abs(float(error)), 1.0)))
    return float(linear_x), float(angular_z), float(integral)


def _compute_search_command(dt: float, config: dict, search_state: SearchState | None):
    state = search_state or _default_search_state(config)
    spin_speed = abs(float(config['search_spin_angular_speed']))
    spin_target = abs(float(config['search_spin_revolution_target']))
    arc_linear_speed = abs(float(config['search_arc_linear_speed']))
    arc_angular_speed = abs(float(config['search_arc_angular_speed']))
    arc_target = abs(float(config['search_arc_revolution_target']))

    if state.mode in ('idle', 'spin'):
        progress = state.progress + spin_speed * dt
        if progress >= spin_target:
            next_state = SearchState(mode='arc', progress=0.0, arc_direction=state.arc_direction)
            return arc_linear_speed, arc_angular_speed * next_state.arc_direction, next_state
        return 0.0, spin_speed, SearchState(mode='spin', progress=progress, arc_direction=state.arc_direction)

    progress = state.progress + arc_angular_speed * dt
    if progress >= arc_target:
        next_direction = -_sign(state.arc_direction)
        next_state = SearchState(mode='arc', progress=0.0, arc_direction=next_direction)
        return arc_linear_speed, arc_angular_speed * next_state.arc_direction, next_state
    return arc_linear_speed, arc_angular_speed * state.arc_direction, SearchState(mode='arc', progress=progress, arc_direction=state.arc_direction)


def compute_control_output(error, prev_error: float, integral: float, dt: float, config: dict, search_state: SearchState | None = None):
    if error is not None:
        linear_x, angular_z, integral = compute_control_command(
            error=error,
            prev_error=prev_error,
            integral=integral,
            dt=dt,
            config=config,
        )
        return linear_x, angular_z, integral, _default_search_state(config)

    if not bool(config.get('search_enabled', False)):
        return 0.0, 0.0, 0.0, _default_search_state(config)

    linear_x, angular_z, next_state = _compute_search_command(
        dt=dt,
        config=config,
        search_state=search_state,
    )
    return float(linear_x), float(angular_z), 0.0, next_state


if rclpy is not None:
    class LaneControllerNode(Node):
        def __init__(self) -> None:
            super().__init__('lane_controller_node')
            self.declare_parameter('error_topic', '/lane_tracking/error')
            self.declare_parameter('cmd_vel_topic', '/cmd_vel')
            self.declare_parameter('control_period', 0.1)
            self.declare_parameter('error_timeout_sec', 0.4)
            self.declare_parameter('kp', 1.2)
            self.declare_parameter('ki', 0.08)
            self.declare_parameter('kd', 0.05)
            self.declare_parameter('integral_limit', 1.5)
            self.declare_parameter('base_linear_speed', 0.25)
            self.declare_parameter('min_linear_speed', 0.05)
            self.declare_parameter('max_angular_speed', 1.2)
            self.declare_parameter('speed_reduction_gain', 0.6)
            self.declare_parameter('search_enabled', True)
            self.declare_parameter('search_spin_angular_speed', 0.35)
            self.declare_parameter('search_spin_revolution_target', 6.283185307179586)
            self.declare_parameter('search_arc_linear_speed', 0.08)
            self.declare_parameter('search_arc_angular_speed', 0.20)
            self.declare_parameter('search_arc_revolution_target', 6.283185307179586)
            self.declare_parameter('search_default_turn_direction', 1.0)

            error_topic = self.get_parameter('error_topic').value
            cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
            self.control_period = float(self.get_parameter('control_period').value)
            self.error_timeout_sec = float(self.get_parameter('error_timeout_sec').value)

            self.error_subscription = self.create_subscription(Float32, error_topic, self.error_cb, 10)
            self.cmd_publisher = self.create_publisher(Twist, cmd_vel_topic, 10)
            self.timer = self.create_timer(self.control_period, self.on_timer)

            self.latest_error = None
            self.prev_error = 0.0
            self.integral = 0.0
            self.last_error_time = None
            self.search_state = _default_search_state(self.config_dict())

        def config_dict(self) -> dict:
            return {
                'kp': self.get_parameter('kp').value,
                'ki': self.get_parameter('ki').value,
                'kd': self.get_parameter('kd').value,
                'integral_limit': self.get_parameter('integral_limit').value,
                'base_linear_speed': self.get_parameter('base_linear_speed').value,
                'min_linear_speed': self.get_parameter('min_linear_speed').value,
                'max_angular_speed': self.get_parameter('max_angular_speed').value,
                'speed_reduction_gain': self.get_parameter('speed_reduction_gain').value,
                'search_enabled': self.get_parameter('search_enabled').value,
                'search_spin_angular_speed': self.get_parameter('search_spin_angular_speed').value,
                'search_spin_revolution_target': self.get_parameter('search_spin_revolution_target').value,
                'search_arc_linear_speed': self.get_parameter('search_arc_linear_speed').value,
                'search_arc_angular_speed': self.get_parameter('search_arc_angular_speed').value,
                'search_arc_revolution_target': self.get_parameter('search_arc_revolution_target').value,
                'search_default_turn_direction': self.get_parameter('search_default_turn_direction').value,
            }

        def error_cb(self, msg: Float32) -> None:
            self.latest_error = float(msg.data)
            self.last_error_time = self.get_clock().now()

        def on_timer(self) -> None:
            error = self.latest_error
            if self.last_error_time is None:
                error = None
            else:
                age = (self.get_clock().now() - self.last_error_time).nanoseconds / 1e9
                if age > self.error_timeout_sec or not math.isfinite(self.latest_error):
                    error = None

            linear_x, angular_z, self.integral, self.search_state = compute_control_output(
                error=error,
                prev_error=self.prev_error,
                integral=self.integral,
                dt=self.control_period,
                config=self.config_dict(),
                search_state=self.search_state,
            )
            if error is not None:
                self.prev_error = float(error)
            else:
                self.prev_error = 0.0

            twist = Twist()
            twist.linear.x = float(linear_x)
            twist.angular.z = float(angular_z)
            self.cmd_publisher.publish(twist)


def main() -> None:
    if rclpy is None:
        raise RuntimeError('ROS 2 Python dependencies are unavailable; source the ROS environment first')
    rclpy.init()
    node = LaneControllerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
