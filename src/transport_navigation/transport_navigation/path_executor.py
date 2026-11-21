"""Explicit whole-route Action server, using the STM32 ExecuteMotion interface."""
import math
import signal
import time

import rclpy
from action_msgs.msg import GoalStatus
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.task import Future
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformListener
from transport_interfaces.action import ExecuteMotion, ExecuteRoute
from transport_interfaces.msg import ConnectionStatus, ProcessedRoute
from transport_interfaces.srv import StopBase

from .route_execution import (build_steps, collision_check, distance, line_error,
                             map_digest, point_of, to_map, validate_route, wrap, yaw_of)


class PathExecutor(Node):
    def __init__(self):
        super().__init__('path_executor')
        defaults = dict(robot_radius=0.0, safety_margin=0.00, tracking_tolerance=0.05,
                        position_tolerance=0.03, yaw_tolerance=0.03,
                        distance_tolerance=0.005, linear_speed=0.15, linear_acceleration=0.2,
                        angular_speed=0.5, angular_acceleration=0.5,
                        feedback_timeout=0.6, response_timeout=3.0, stop_timeout=5.0,
                        route_timeout=600.0, max_segment_length=10.0)
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.cfg = {name: self.get_parameter(name).value for name in defaults}
        for name, value in self.cfg.items():
            if not math.isfinite(value) or value < 0 or (name != 'safety_margin' and value == 0):
                raise ValueError(f'{name} must be positive (set actual robot_radius explicitly)')
        if self.cfg['position_tolerance'] > self.cfg['tracking_tolerance']:
            raise ValueError('position_tolerance must not exceed tracking_tolerance')
        self.group = ReentrantCallbackGroup()
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.route = self.grid = self.grid_hash = self.odom = self.connection = None
        self.map_ready = False
        self.odom_at = self.connection_at = -1e9
        self.run = None
        self.prepared = None
        self.used = set()
        self.locked = False
        self.state_text = ''
        self.last_feedback = -1e9
        self.status_pub = self.create_publisher(String, '/navigation/execution_status', qos)
        self.create_subscription(ProcessedRoute, '/navigation/processed_route', self.on_route, qos)
        self.create_subscription(OccupancyGrid, '/navigation/mission_map', self.on_map, qos)
        self.create_subscription(Bool, '/navigation/map_ready', self.on_map_ready, qos)
        self.create_subscription(Odometry, '/odom', self.on_odom, 5)
        self.create_subscription(ConnectionStatus, '/base/connection_status', self.on_connection, qos)
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.motion_client = ActionClient(self, ExecuteMotion, '/base/execute_motion', callback_group=self.group)
        self.stop_client = self.create_client(StopBase, '/base/stop', callback_group=self.group)
        self.server = ActionServer(self, ExecuteRoute, '/navigation/execute_route',
                                   execute_callback=self.execute, goal_callback=self.on_goal,
                                   handle_accepted_callback=self.on_accepted,
                                   cancel_callback=self.on_cancel, callback_group=self.group)
        self.timer = self.create_timer(0.05, self.tick, callback_group=self.group,
                                      clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.status('WAITING: waiting for route, map, odometry, bridge and map->odom')

    def status(self, text):
        if text != self.state_text:
            self.state_text = text
            self.status_pub.publish(String(data=text))
            self.get_logger().info(text)

    def invalidate(self, reason):
        self.route = None
        if self.run:
            self.halt(self.run, reason)
        else:
            self.status('WAITING: ' + reason)

    def on_route(self, msg):
        try:
            route = validate_route(msg, max_length=self.cfg['max_segment_length'])
            if self.run and route != self.run['route']:
                self.halt(self.run, 'route changed during execution; replan after stopping')
            self.route = route
            if not self.run:
                self.status('ROUTE_RECEIVED: ' + route.route_id + '; waiting for explicit ExecuteRoute goal')
        except ValueError as exc:
            self.invalidate(str(exc))

    def on_map(self, msg):
        try:
            digest = map_digest(msg)
        except ValueError as exc:
            self.grid = self.grid_hash = None
            self.invalidate(str(exc))
            return
        if self.grid_hash is not None and digest != self.grid_hash:
            self.invalidate('map changed; request a new plan')
        self.grid, self.grid_hash = msg, digest

    def on_map_ready(self, msg):
        self.map_ready = msg.data
        if not msg.data:
            self.invalidate('map not ready')

    def on_odom(self, msg):
        try:
            if msg.header.frame_id != 'odom' or msg.child_frame_id != 'base_link':
                raise ValueError('expected odom/base_link frames')
            xy = point_of(msg.pose.pose.position)
            yaw = yaw_of(msg.pose.pose.orientation)
            v, w = msg.twist.twist.linear.x, msg.twist.twist.angular.z
            stamp = Time.from_msg(msg.header.stamp)
            age = (self.get_clock().now()-stamp).nanoseconds / 1e9
            if not all(math.isfinite(x) for x in (v, w)) or age < -0.1 or age > self.cfg['feedback_timeout']:
                raise ValueError('invalid or stale odometry')
            self.odom = (*xy, yaw, v, w)
            self.odom_at = time.monotonic()
        except ValueError:
            self.odom = None

    def on_connection(self, msg):
        self.connection = msg
        self.connection_at = time.monotonic()

    def transform(self):
        tf = self.tf.lookup_transform('map', 'odom', Time())
        xy = point_of(tf.transform.translation)
        yaw = yaw_of(tf.transform.rotation)
        if tf.header.stamp.sec or tf.header.stamp.nanosec:
            age = (self.get_clock().now()-Time.from_msg(tf.header.stamp)).nanoseconds / 1e9
            if age > self.cfg['feedback_timeout'] or age < -0.1:
                raise ValueError('map->odom transform is stale')
        return (*xy, yaw)

    def health(self, now):
        if self.locked:
            raise ValueError('executor locked after unconfirmed stop; inspect and restart')
        if not self.map_ready or self.grid is None:
            raise ValueError('mission map not ready')
        if self.odom is None or now-self.odom_at > self.cfg['feedback_timeout']:
            raise ValueError('odometry missing or stale')
        c = self.connection
        if (c is None or now-self.connection_at > self.cfg['feedback_timeout']
                or not c.port_open or not c.session_bound or c.motion_locked
                or c.odometry_age < 0 or c.odometry_age > self.cfg['feedback_timeout']
                or c.status_age < 0 or c.status_age > self.cfg['feedback_timeout']):
            raise ValueError('bridge missing, stale, unbound or locked')
        return self.transform()

    def on_goal(self, goal):
        try:
            if self.run or self.prepared:
                raise ValueError('another route is executing')
            if not self.route or goal.route_id != self.route.route_id:
                raise ValueError('route_id must match current valid processed route')
            if goal.route_id in self.used or len(self.used) >= 2048:
                raise ValueError('route already attempted or history full; obtain new route/restart')
            tf = self.health(time.monotonic())
            if not self.connection.ready or not self.motion_client.server_is_ready() or not self.stop_client.service_is_ready():
                raise ValueError('motion/stop server or idle bridge unavailable')
            pose = to_map(self.odom[:3], tf)
            if distance(pose, self.route.points[0]) > self.cfg['position_tolerance']:
                raise ValueError('current position is not near route start')
            radius = self.cfg['robot_radius'] + self.cfg['safety_margin']
            collision_check(self.route, self.grid, radius)
            steps = build_steps(self.route, tf[2])
            for step in steps:
                self.make_motion(step)  # Reject unexecutable timeout/length before starting.
            self.prepared = dict(route=self.route, tf=tf, map_hash=self.grid_hash, steps=steps,
                                 boot=self.connection.boot_id, session=self.connection.session_id)
            return GoalResponse.ACCEPT
        except Exception as exc:
            self.get_logger().warning('Route rejected: ' + str(exc))
            return GoalResponse.REJECT

    def on_accepted(self, handle):
        run = self.prepared
        self.prepared = None
        run.update(handle=handle, future=Future(), index=0, completed=0,
                   started=time.monotonic(), wait_since=time.monotonic(), phase='WAIT_READY',
                   child=None, pending=False, child_settled=True, child_stopped=True,
                   halt_reason=None, cancel=False, stop_future=None, stop_seen=False,
                   stop_id=0, finished=False)
        self.run = run
        self.used.add(run['route'].route_id)
        handle.execute()

    async def execute(self, handle):
        run = self.run
        result, outcome = await run['future']
        if outcome == 'success':
            handle.succeed()
        elif outcome == 'canceled' and handle.is_cancel_requested:
            handle.canceled()
        else:
            handle.abort()
        if self.run is run:
            self.run = None
        return result

    def on_cancel(self, handle):
        if not self.run or self.run['handle'] is not handle or self.run['finished']:
            return CancelResponse.REJECT
        self.run['cancel'] = True
        self.halt(self.run, 'route canceled by client')
        return CancelResponse.ACCEPT

    def send_stop(self, run):
        if self.stop_client.service_is_ready():
            request = StopBase.Request()
            request.reason = 1 if run.get('cancel') else 2
            run['stop_future'] = self.stop_client.call_async(request)

    def halt(self, run, reason):
        if run['finished']:
            return
        if run['halt_reason'] is None:
            run['halt_reason'] = reason
            run['halt_at'] = time.monotonic()
            run['phase'] = 'STOPPING'
            self.send_stop(run)
            if run['child'] is not None and not run['child_settled']:
                run['child'].cancel_goal_async()
            self.status('STOPPING: ' + reason)

    def make_motion(self, step):
        goal = ExecuteMotion.Goal()
        goal.motion_type = step.kind
        goal.target_yaw = step.yaw
        goal.target_distance = step.length
        goal.max_linear_velocity = self.cfg['linear_speed']
        goal.linear_acceleration = self.cfg['linear_acceleration']
        goal.max_angular_velocity = self.cfg['angular_speed']
        goal.angular_acceleration = self.cfg['angular_acceleration']
        goal.distance_tolerance = min(self.cfg['distance_tolerance'], step.length/2) if step.kind == 2 else self.cfg['distance_tolerance']
        goal.yaw_tolerance = self.cfg['yaw_tolerance']
        duration = (step.length/self.cfg['linear_speed'] + self.cfg['linear_speed']/self.cfg['linear_acceleration']) if step.kind == 2 else (math.pi/self.cfg['angular_speed'] + self.cfg['angular_speed']/self.cfg['angular_acceleration'])
        goal.timeout_ms = math.ceil((2*duration + 3)*1000)
        if goal.timeout_ms > 120000:
            raise ValueError('segment timeout exceeds bridge default 120 s; shorten segment')
        return goal

    def send_step(self, run, now):
        step = run['steps'][run['index']]
        run.update(phase=step.phase, pending=True, child=None, child_settled=False,
                   child_stopped=False, sent_at=now)
        request = self.make_motion(step)
        run['child_deadline'] = now + request.timeout_ms/1000 + self.cfg['response_timeout']
        future = self.motion_client.send_goal_async(request)
        future.add_done_callback(lambda f: self.child_accepted(run, f))
        self.status(f'{step.phase}: segment {step.segment}/{len(run["route"].lengths)}')

    def child_accepted(self, run, future):
        run['pending'] = False
        try:
            handle = future.result()
            if not handle.accepted:
                run.update(child_settled=True, child_stopped=True)
                self.halt(run, 'bridge rejected motion')
                return
            run['child'] = handle
            handle.get_result_async().add_done_callback(lambda f: self.child_result(run, f))
            if run['halt_reason'] or run['finished']:
                # Also handles a late acceptance after route-level timeout.
                handle.cancel_goal_async()
                self.send_stop(run)
        except Exception as exc:
            self.halt(run, 'motion acceptance error: ' + str(exc))

    def child_result(self, run, future):
        try:
            wrapped = future.result()
            run['child_settled'] = True
            run['child_stopped'] = wrapped.result.stopped_confirmed
            if run['finished']:
                return
            if run['halt_reason']:
                return
            if wrapped.status != GoalStatus.STATUS_SUCCEEDED or not wrapped.result.success or not wrapped.result.stopped_confirmed:
                self.halt(run, 'motion failed: ' + wrapped.result.message)
                return
            run['phase'] = 'VERIFY_ENDPOINT'
            run['wait_since'] = time.monotonic()
        except Exception as exc:
            self.halt(run, 'motion result error: ' + str(exc))

    def finish(self, run, success, stopped, message):
        if run['finished']:
            return
        result = ExecuteRoute.Result()
        result.success, result.stopped_confirmed = success, stopped
        result.completed_segments, result.message = run['completed'], message
        run['finished'] = True
        outcome = 'success' if success else 'canceled' if run['cancel'] and stopped else 'failed'
        self.status(outcome.upper() + ': ' + message)
        run['future'].set_result((result, outcome))

    def tick_stop(self, run, now):
        future = run['stop_future']
        if future is None:
            self.send_stop(run)
        elif future.done():
            try:
                response = future.result()
                if response.submitted:
                    run['stop_id'] = response.command_id
            except Exception:
                pass
        c = self.connection
        fresh = c is not None and now-self.connection_at < self.cfg['feedback_timeout']
        if fresh and run['stop_id'] and c.stop_command_id == run['stop_id']:
            run['stop_seen'] = True
        stopped = (fresh and c.port_open and c.session_bound and not c.motion_locked
                   and (c.boot_id, c.session_id) == (run['boot'], run['session'])
                   and self.odom is not None and now-self.odom_at < self.cfg['feedback_timeout']
                   and abs(self.odom[3]) <= 0.015 and abs(self.odom[4]) <= 0.03
                   and c.active_command_id == 0 and c.stop_command_id == 0
                   and c.stop_state == 'SUCCEEDED'
                   and run['child_settled'] and not run['pending']
                   and (run['child_stopped'] or run['stop_seen']))
        if stopped:
            self.finish(run, False, True, run['halt_reason'])
        elif now-run['halt_at'] > self.cfg['stop_timeout']:
            self.locked = True
            self.finish(run, False, False, run['halt_reason'] + '; stop unconfirmed, executor locked')

    def tick(self):
        run = self.run
        if run is None or run['finished']:
            return
        now = time.monotonic()
        if run['halt_reason']:
            self.tick_stop(run, now)
            return
        try:
            tf = self.health(now)
            c = self.connection
            if (c.boot_id, c.session_id) != (run['boot'], run['session']):
                raise ValueError('bridge session/STM32 boot changed')
            if self.grid_hash != run['map_hash'] or self.route != run['route']:
                raise ValueError('map or route changed')
            if distance(tf, run['tf']) > 0.005 or abs(wrap(tf[2]-run['tf'][2])) > 0.01:
                raise ValueError('map->odom alignment changed')
            if now-run['started'] > self.cfg['route_timeout']:
                raise ValueError('route timeout')
            pose = to_map(self.odom[:3], run['tf'])
            step = run['steps'][run['index']]
            error = line_error(pose, step.start, step.end)
            if error > self.cfg['tracking_tolerance']:
                raise ValueError(f'position departed permitted corridor: {error:.3f} m')
            if run['phase'] == 'VERIFY_ENDPOINT':
                # Wait for odometry newer than the child action completion callback.
                if self.odom_at > run['wait_since']:
                    if distance(pose, step.end) > self.cfg['position_tolerance']:
                        raise ValueError('actual endpoint outside position tolerance')
                    if abs(wrap(self.odom[2]-step.yaw)) > self.cfg['yaw_tolerance'] + 1e-4:
                        raise ValueError('actual heading outside tolerance')
                    if step.kind == 2:
                        run['completed'] += 1
                    run['index'] += 1
                    if run['index'] == len(run['steps']):
                        self.finish(run, True, True, 'all segments and final heading verified')
                        return
                    run['phase'], run['wait_since'] = 'WAIT_READY', now
                elif now-run['wait_since'] > self.cfg['feedback_timeout']:
                    raise ValueError('no fresh endpoint odometry')
            elif run['phase'] == 'WAIT_READY':
                if c.ready:
                    if distance(pose, step.start) > self.cfg['position_tolerance']:
                        raise ValueError('not near next segment start')
                    self.send_step(run, now)
                elif now-run['wait_since'] > self.cfg['response_timeout']:
                    raise ValueError('bridge did not become ready')
            elif run['pending'] and now-run['sent_at'] > self.cfg['response_timeout']:
                raise ValueError('motion goal response timeout')
            elif now > run['child_deadline']:
                raise ValueError('motion result timeout')
            if now-self.last_feedback >= 0.2:
                feedback = ExecuteRoute.Feedback()
                feedback.route_id = run['route'].route_id
                feedback.segment_index = step.segment
                feedback.segment_count = len(run['route'].lengths)
                feedback.completed_segments = run['completed']
                feedback.phase = run['phase']
                feedback.position_error = error
                feedback.message = 'executing with odometry feedback'
                run['handle'].publish_feedback(feedback)
                self.last_feedback = now
        except Exception as exc:
            self.halt(run, str(exc))

    def close(self):
        if self.run and not self.run['finished']:
            run = self.run
            self.halt(run, 'executor shutting down')
            deadline = time.monotonic() + self.cfg['stop_timeout'] + 0.2
            while rclpy.ok() and not run['finished'] and time.monotonic() < deadline:
                rclpy.spin_once(self, timeout_sec=0.05)
        self.server.destroy()
        self.motion_client.destroy()
        self.destroy_node()


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    def interrupt(signum, frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGINT, interrupt)
    signal.signal(signal.SIGTERM, interrupt)
    node = None
    try:
        node = PathExecutor()
        rclpy.spin(node)  # Keep single-threaded state ownership.
    except KeyboardInterrupt:
        pass
    finally:
        if node:
            node.close()
        if rclpy.ok():
            rclpy.shutdown()
