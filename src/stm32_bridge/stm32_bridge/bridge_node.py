"""ROS adapter. One executor thread owns state; I/O thread only queues bytes."""
import math
import queue
import time

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.task import Future
from transport_interfaces.action import ExecuteMotion
from transport_interfaces.msg import BaseStatus, ConnectionStatus
from transport_interfaces.srv import StopBase

from .motion_client import MotionClient
from .odometry_publisher import OdometryPublisher
from .protocol import Motion
from .serial_transport import SerialTransport
from .stream_parser import StreamParser


class BridgeNode(Node):
    def __init__(self, **kwargs):
        super().__init__('stm32_bridge', **kwargs)
        defaults = {
            'serial_port': '/dev/ttyUSB0', 'baudrate': 115200,
            'heartbeat_period': 0.1, 'status_timeout': 0.6, 'odom_timeout': 0.3,
            'ack_timeout': 0.25, 'max_retries': 2, 'query_period': 0.4, 'stop_timeout': 4.0,
            'odom_frame': 'odom', 'base_frame': 'base_link', 'publish_tf': True,
            'max_distance': 1.5707964, 'max_speed': 0.20, 'max_acceleration': 0.20,
            'max_angular_speed': 1.5, 'max_angular_acceleration': 1.3962634, 'max_timeout_ms': 120000,
            'pose_variance': [0.01, 0.01, 1000.0, 1000.0, 1000.0, 0.02],
            'twist_variance': [0.01, 1000.0, 1000.0, 1000.0, 1000.0, 0.02],
        }
        for name, default in defaults.items():
            self.declare_parameter(name, default)
        config = {name: self.get_parameter(name).value for name in defaults}
        for name in ('heartbeat_period', 'status_timeout', 'odom_timeout', 'ack_timeout',
                     'query_period', 'stop_timeout', 'max_distance', 'max_speed',
                     'max_acceleration', 'max_angular_speed', 'max_angular_acceleration'):
            if not math.isfinite(config[name]) or config[name] <= 0:
                raise ValueError(f'{name} must be finite and positive')
        if config['heartbeat_period'] >= 0.5 or config['max_retries'] < 0:
            raise ValueError('heartbeat must be faster than device watchdog; retries nonnegative')
        for name in ('pose_variance', 'twist_variance'):
            if len(config[name]) != 6 or any(not math.isfinite(v) or v <= 0 for v in config[name]):
                raise ValueError(f'{name} must have six positive variances')
        if not config['odom_frame'] or not config['base_frame'] or config['odom_frame'] == config['base_frame']:
            raise ValueError('odom_frame and base_frame must be different nonempty frames')
        self.parser = StreamParser()
        self.transport = SerialTransport(config['serial_port'], config['baudrate'])
        self.engine = MotionClient(self.transport.send,
            **{key: config[key] for key in ('heartbeat_period', 'status_timeout', 'odom_timeout',
                                           'ack_timeout', 'max_retries', 'query_period', 'stop_timeout')},
            limits={key: config['max_' + key] for key in ('distance', 'speed', 'acceleration',
                                                         'angular_speed', 'angular_acceleration', 'timeout_ms')})
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.base_pub = self.create_publisher(BaseStatus, '/base/status', qos)
        self.connection_pub = self.create_publisher(ConnectionStatus, '/base/connection_status', qos)
        self.odometry = OdometryPublisher(self, config['odom_frame'], config['base_frame'],
                                         config['publish_tf'], config['pose_variance'], config['twist_variance'])
        self.reserved = False
        self.active_goal_handle = self.result_future = None
        self.last_feedback = None
        self.odom_version = self.status_version = 0
        self.status_publish_time = -1e9
        self.last_connection_state = ''
        self.group = ReentrantCallbackGroup()
        self.action = ActionServer(self, ExecuteMotion, '/base/execute_motion',
                                   execute_callback=self.execute,
                                   goal_callback=self.on_goal,
                                   handle_accepted_callback=self.on_accepted,
                                   cancel_callback=self.on_cancel,
                                   callback_group=self.group)
        self.stop_service = self.create_service(StopBase, '/base/stop', self.on_stop,
                                                callback_group=self.group)
        self.timer = self.create_timer(0.02, self.tick, callback_group=self.group,
                                      clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.get_logger().info(f'Opening {config["serial_port"]}; waiting for session and odometry')

    @staticmethod
    def motion_from_goal(g):
        return Motion(g.motion_type, g.target_yaw, distance=g.target_distance,
                      speed=g.max_linear_velocity, acceleration=g.linear_acceleration,
                      angular_speed=g.max_angular_velocity, angular_acceleration=g.angular_acceleration,
                      distance_tolerance=g.distance_tolerance, yaw_tolerance=g.yaw_tolerance,
                      timeout_ms=g.timeout_ms)

    def on_goal(self, request):
        try:
            self.motion_from_goal(request).validate(self.engine.limits)
            if self.reserved or not self.engine.ready(time.monotonic()):
                raise ValueError('bridge not ready or another goal is active')
        except (ValueError, OverflowError) as exc:
            self.get_logger().warning(f'Goal rejected: {exc}')
            return GoalResponse.REJECT
        self.reserved = True
        return GoalResponse.ACCEPT

    def on_accepted(self, handle):
        self.active_goal_handle = handle
        self.result_future = Future()
        self.last_feedback = None
        try:
            command = self.engine.begin(self.motion_from_goal(handle.request), time.monotonic())
            self.get_logger().info(f'Goal {bytes(handle.goal_id.uuid).hex()} -> command {command}')
        except (ValueError, ConnectionError, OverflowError) as exc:
            if self.engine.motion:
                self.io_failed(str(exc))
            else:
                self.result_future.set_result(dict(command_id=0, outcome='failed', bridge_error=7,
                    stm32_error=0, stopped_confirmed=False, message=str(exc)))
        handle.execute()

    async def execute(self, handle):
        future = self.result_future
        try:
            outcome = await future
            result = ExecuteMotion.Result()
            for field in ('command_id', 'bridge_error', 'stm32_error', 'stopped_confirmed', 'message'):
                setattr(result, field, outcome[field])
            result.success = outcome['outcome'] == 'succeeded'
            if result.success:
                handle.succeed()
            elif outcome['outcome'] == 'canceled' and handle.is_cancel_requested:
                handle.canceled()
            else:
                handle.abort()
            return result
        finally:
            self.active_goal_handle = self.result_future = None
            self.reserved = False

    def on_cancel(self, handle):
        if self.active_goal_handle is not handle or self.engine.motion is None:
            return CancelResponse.REJECT
        try:
            self.engine.request_stop(time.monotonic(), cancel=True)
            return CancelResponse.ACCEPT
        except (ValueError, ConnectionError) as exc:
            self.io_failed(str(exc))
            return CancelResponse.REJECT

    def on_stop(self, request, response):
        try:
            response.command_id = self.engine.request_stop(time.monotonic(), request.reason)
            response.submitted = True
            response.message = 'STOP queued; wait for stopped state, not just this response'
        except (ValueError, ConnectionError) as exc:
            response.submitted = False
            response.message = str(exc)
            if isinstance(exc, ConnectionError):
                self.io_failed(str(exc))
        return response

    def io_failed(self, reason):
        self.engine.disconnect(reason, time.monotonic())
        self.transport.reset()

    def tick(self):
        now = time.monotonic()
        try:
            # Bound each callback's work; aged events remain aged, not stamped now.
            for _ in range(128):
                try:
                    kind, data, received_at = self.transport.events.get_nowait()
                except queue.Empty:
                    break
                if kind == 'opened':
                    self.parser = StreamParser()
                    self.engine.open(received_at)
                elif kind == 'error':
                    self.engine.disconnect(data, received_at)
                else:
                    for frame in self.parser.feed(data, received_at):
                        self.engine.receive(frame, received_at)
            for frame in self.parser.feed(b'', now):
                self.engine.receive(frame, now)
            self.engine.tick(now)
        except (ConnectionError, ValueError, OverflowError) as exc:
            self.io_failed(str(exc))

        while self.engine.outcomes:
            result = self.engine.outcomes.pop(0)
            self.get_logger().info(f'Command {result["command_id"]}: {result["outcome"]}, {result["message"]}')
            if self.result_future is not None and not self.result_future.done():
                self.result_future.set_result(result)
        pending = self.engine.motion
        if self.active_goal_handle and pending and pending.feedback and pending.feedback != self.last_feedback:
            v = pending.feedback
            feedback = ExecuteMotion.Feedback()
            feedback.command_id = pending.command
            feedback.motion_state, feedback.valid_flags, feedback.elapsed_ms = v[1], v[3], v[5]
            (feedback.traveled_distance, feedback.remaining_distance, feedback.current_yaw,
             feedback.yaw_error, feedback.linear_velocity, feedback.angular_velocity) = (
                 float(v[7]), float(v[8]), float(v[10]), float(v[11]), float(v[12]), float(v[13]))
            self.active_goal_handle.publish_feedback(feedback)
            self.last_feedback = v
        if self.engine.odom_version != self.odom_version and self.engine.fresh(now):
            self.odometry.publish(self.engine.odom)
            self.odom_version = self.engine.odom_version
        if self.engine.status_version != self.status_version and self.engine.status:
            v = self.engine.status
            msg = BaseStatus()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.boot_id, msg.session_id = self.engine.session.boot, self.engine.session.session
            (msg.uptime_ms, msg.base_state, msg.active_motion_type, msg.status_flags,
             msg.active_command_id, msg.fault_flags, msg.battery_voltage, msg.valid_flags) = v[:8]
            self.base_pub.publish(msg)
            self.status_version = self.engine.status_version
        if now - self.status_publish_time >= 0.1:
            self.publish_connection(now)
            self.status_publish_time = now

    def publish_connection(self, now):
        e = self.engine
        msg = ConnectionStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.port_open, msg.session_bound, msg.ready = e.port_open, e.session.bound, e.ready(now) and not self.reserved
        msg.motion_locked = e.locked
        msg.boot_id, msg.session_id = e.session.boot, e.session.session
        msg.status_age = max(0.0, now - e.session.status_time) if e.status else -1.0
        msg.odometry_age = max(0.0, now - e.session.odom_time) if e.odom else -1.0
        msg.parser_errors = self.parser.errors & 0xffffffff
        msg.active_command_id = e.motion.command if e.motion else 0
        msg.stop_command_id = e.stop.command if e.stop else 0
        msg.stop_state = e.stop_state
        msg.state = ('DISCONNECTED' if not e.port_open else 'LOCKED' if e.locked else
                     'HANDSHAKING' if not e.session.bound else 'STALE' if not e.fresh(now) else
                     'STOPPING' if e.stop else 'BUSY' if self.reserved else 'READY' if msg.ready else 'NOT_READY')
        msg.message = e.message
        self.connection_pub.publish(msg)
        if msg.state != self.last_connection_state:
            self.get_logger().info(f'Connection: {msg.state} ({msg.message})')
            self.last_connection_state = msg.state

    def close(self):
        # Best effort graceful STOP. Watchdog is still required for kill/power loss.
        if self.engine.port_open:
            try:
                self.engine.request_stop(time.monotonic(), 3)
                time.sleep(0.1)
            except (ValueError, ConnectionError):
                pass
        self.transport.close()
        self.action.destroy()
        self.destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = BridgeNode()
        rclpy.spin(node)  # SingleThreadedExecutor is intentional: serial state ownership.
    except KeyboardInterrupt:
        pass
    finally:
        if node:
            node.close()
        if rclpy.ok():
            rclpy.shutdown()
