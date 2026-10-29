"""在现有连接节点上增加单次扫描；仅使用单线程执行器。"""
import time

import rclpy
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.task import Future
from radar_interfaces.action import ScanOnce
from radar_interfaces.msg import Obstacle, ObstacleArray

from .radar_node import RadarNode
from .client import LinkState, RequestRejected


class ScanNode(RadarNode):
    """串口仍由基类定时器独占；Action协程等待Future时让出执行器。"""

    def __init__(self):
        super().__init__()
        self._declare_read_only('frame_id', 'radar_field', '雷达场地坐标系，不自动生成TF')
        self._frame_id = self.get_parameter('frame_id').value
        if not self._frame_id.strip():
            raise ValueError('frame_id不能为空')
        self._reserved = False
        self._goal = None
        self._completion = None
        self._cancel_pending = False
        self._cancel_deadline = None
        self._publisher = self.create_publisher(ObstacleArray, 'radar/obstacles', 10)
        self._action_group = ReentrantCallbackGroup()
        self._action = ActionServer(
            self, ScanOnce, 'radar/scan_once', self._execute,
            goal_callback=self._accept,
            cancel_callback=self._cancel,
            callback_group=self._action_group,
        )

    def _accept(self, request):
        client = self._client
        if (self._closing or self._reserved or self._stop_deadline is not None
                or client is None or not self._transport.is_open
                or not client.ready or client.busy):
            return GoalResponse.REJECT
        # 在goal回调中预留，避免execute协程启动前接收第二个目标。
        self._reserved = True
        return GoalResponse.ACCEPT

    def _cancel(self, goal):
        if self._completion is None or self._completion.done():
            return CancelResponse.REJECT
        if self._goal is not goal:
            return CancelResponse.REJECT
        self._cancel_pending = True
        self._cancel_deadline = time.monotonic() + 0.5
        try:
            if self._client is not None:
                self._client.request_stop()
        except RequestRejected:
            pass  # 已有STOP在途时继续等待；超时不能宣称停止成功。
        return CancelResponse.ACCEPT

    async def _execute(self, goal):
        self._goal = goal
        self._completion = Future()
        self._cancel_pending = False
        self._cancel_deadline = None
        try:
            if self._closing or self._client is None:
                self._finish(False, '连接不可用')
            else:
                try:
                    self._client.request_scan_once()
                except RequestRejected as exc:
                    self._finish(False, str(exc))
            # 不使用time.sleep或asyncio轮询，不阻塞串口定时器。
            result = await self._completion
            if goal.is_cancel_requested:
                goal.canceled()
            elif result.success:
                goal.succeed()
            else:
                goal.abort()
            return result
        finally:
            self._goal = None
            self._completion = None
            self._cancel_pending = False
            self._cancel_deadline = None
            self._reserved = False

    def _finish(self, success, message, scan=None, stop_confirmed=False):
        if self._completion is None or self._completion.done():
            return
        result = ScanOnce.Result()
        result.success = success
        result.stop_confirmed = stop_confirmed
        result.message = message
        if scan is not None:
            result.scan = scan
        self._completion.set_result(result)

    def _tick(self):
        super()._tick()
        if (self._cancel_pending and self._cancel_deadline is not None
                and time.monotonic() >= self._cancel_deadline):
            self._finish(False, '已取消等待，但未确认雷达停止')

    def _handle_transport_failure(self, reason):
        self._finish(False, '串口中断：' + reason)
        super()._handle_transport_failure(reason)

    def _start_stop_before_close(self, reason):
        self._finish(False, reason + '；尚未确认停止')
        super()._start_stop_before_close(reason)

    def _handle_event(self, event):
        super()._handle_event(event)
        if self._completion is None or self._completion.done():
            return
        kind, data = event.kind, event.data
        if kind == 'status' and data.mode == 1 and not self._cancel_pending:
            feedback = ScanOnce.Feedback()
            feedback.frame_count = data.frame_count
            feedback.total_frames = 14
            feedback.phase = 'scanning'
            self._goal.publish_feedback(feedback)
        elif kind == 'scan' and not self._cancel_pending:
            msg = ObstacleArray()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self._frame_id
            msg.scan_sequence = data.scan_sequence
            msg.frame_count = data.frame_count
            for item in data.obstacles:
                obstacle = Obstacle()
                obstacle.x = item.x_mm / 1000.0
                obstacle.y = item.y_mm / 1000.0
                obstacle.grid_id = item.grid_id
                obstacle.frame_hits = item.frame_hits
                obstacle.confidence_percent = item.confidence_percent
                obstacle.point_count = item.point_count
                msg.obstacles.append(obstacle)
            self._publisher.publish(msg)
            self._finish(True, '完整扫描成功', msg)
        elif kind == 'stopped':
            self._finish(False, '扫描已停止', stop_confirmed=True)
        elif kind == 'fault' and data.kind != 'frame':
            self._finish(False, data.message)
        elif kind == 'link' and data == LinkState.WAIT_HELLO:
            self._finish(False, '链路重新握手，本次扫描失效')


def main(args=None):
    rclpy.init(args=args)
    node = None
    executor = SingleThreadedExecutor()
    try:
        node = ScanNode()
        executor.add_node(node)
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.close()
        executor.shutdown()
        if node is not None:
            node._action.destroy()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
