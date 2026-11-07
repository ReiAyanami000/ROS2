"""Jazzy 静止规划客户端：校验条件、异步请求 Nav2、发布路径。

本版不发送电机命令。地图在本次启动中固定；改变地图需重启整个测试启动文件。
"""

import copy
import math
import time

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import ComputePathToPose
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformException, TransformListener

from transport_navigation.planning_checks import (
    costmap_contains_map, map_digest, point_issue, valid_map,
)


class NavigationManager(Node):
    """把 /goal_pose 转换为 ComputePathToPose Action 请求。"""

    def __init__(self):
        super().__init__('navigation_manager')
        self.declare_parameter('planning_timeout', 10.0)
        self.timeout = float(self.get_parameter('planning_timeout').value)
        if not math.isfinite(self.timeout) or self.timeout <= 0:
            raise ValueError('planning_timeout 必须为正数')

        latched = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.path_pub = self.create_publisher(Path, '/navigation/path', latched)
        self.status_pub = self.create_publisher(
            String, '/navigation/planning_status', latched)
        self.map_sub = self.create_subscription(
            OccupancyGrid, '/navigation/mission_map', self.on_map, latched)
        self.ready_sub = self.create_subscription(
            Bool, '/navigation/map_ready', self.on_map_ready, latched)
        self.costmap_sub = self.create_subscription(
            OccupancyGrid, '/global_costmap/costmap', self.on_costmap, latched)
        self.goal_sub = self.create_subscription(
            PoseStamped, '/goal_pose', self.on_goal, 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.planner = ActionClient(self, ComputePathToPose, '/compute_path_to_pose')
        self.lifecycle = self.create_client(GetState, '/planner_server/get_state')

        self.map = None
        self.map_hash = None
        self.map_ready = False
        self.costmap = None
        self.costmap_time = 0.0
        self.planner_active = False
        self.state_time = 0.0
        self.state_future = None
        self.last_state_query = 0.0
        self.lock_reason = ''
        self.status = None
        self.active_request = None
        self.request_number = 0
        self.goal_handle = None
        self.request_time = 0.0
        self.has_path = False
        self.clear_path()
        self.set_status('WAIT_MAP: 等待有效的合成地图')
        self.timer = self.create_timer(0.2, self.tick)

    def set_status(self, message):
        if message != self.status:
            self.status = message
            self.status_pub.publish(String(data=message))
            self.get_logger().info(message)

    def clear_path(self):
        message = Path()
        message.header.frame_id = 'map'
        message.header.stamp = self.get_clock().now().to_msg()
        self.path_pub.publish(message)
        self.has_path = False

    def lock_session(self, reason):
        """测试版在地图变化或超时后禁止旧结果重新生效。"""
        self.lock_reason = reason
        self.active_request = None
        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()
            self.goal_handle = None
        self.clear_path()
        self.set_status('RESTART_REQUIRED: ' + reason)

    def on_map(self, message):
        if not valid_map(message):
            self.lock_session('地图不合法；修正后重启 planning_test.launch.py')
            return
        signature = map_digest(message)
        if self.map_hash is not None and signature != self.map_hash:
            self.lock_session('地图内容改变；本版需重启整个规划测试')
            return
        self.map = message
        self.map_hash = signature

    def on_map_ready(self, message):
        if self.map_ready and not message.data:
            self.lock_session('合成地图失效或开始更新；重启整个规划测试')
        self.map_ready = message.data

    def on_costmap(self, message):
        self.costmap = message
        self.costmap_time = time.monotonic()

    def on_lifecycle_state(self, future):
        self.state_future = None
        try:
            self.planner_active = future.result().current_state.id == State.PRIMARY_STATE_ACTIVE
            self.state_time = time.monotonic()
        except Exception as error:
            self.planner_active = False
            self.get_logger().warning(f'读取规划器状态失败：{error}')

    def tick(self):
        """周期检查只安排异步操作，不等待服务或 Action 完成。"""
        now = time.monotonic()
        if (self.state_future is None and now - self.last_state_query >= 1.0
                and self.lifecycle.service_is_ready()):
            self.last_state_query = now
            self.state_future = self.lifecycle.call_async(GetState.Request())
            self.state_future.add_done_callback(self.on_lifecycle_state)
        elif self.state_future is not None and now - self.last_state_query > 3.0:
            self.state_future.cancel()
            self.state_future = None
            self.planner_active = False

        if self.lock_reason:
            return
        if self.active_request is not None:
            if now - self.request_time > self.timeout:
                self.lock_session('规划超时，已请求取消；重启整个规划测试后重试')
            return
        reason = self.readiness_issue()
        if reason:
            if self.has_path:
                self.clear_path()
            self.set_status(reason)
        elif not self.has_path and not self.status.startswith(('FAILED:', 'REJECTED:')):
            self.set_status('READY: 在 RViz 使用 2D Goal Pose 选择终点')

    def readiness_issue(self):
        if self.lock_reason:
            return 'RESTART_REQUIRED: ' + self.lock_reason
        if self.map is None or not self.map_ready:
            return 'WAIT_MAP: 等待本轮有效地图和 map_ready=true'
        try:
            transform = self.tf_buffer.lookup_transform('map', 'base_link', Time())
        except TransformException:
            return 'WAIT_TF: 缺少 map 到 base_link 的变换'
        p = transform.transform.translation
        issue = point_issue(self.map, p.x, p.y)
        if issue:
            return 'WAIT_START: 起点' + issue
        now = time.monotonic()
        if (not self.planner_active or now - self.state_time > 3.0
                or not self.planner.server_is_ready()):
            return 'WAIT_PLANNER: 等待 planner_server 激活'
        if (self.costmap is None or now - self.costmap_time > 3.0
                or not costmap_contains_map(self.map, self.costmap)):
            return 'WAIT_COSTMAP: 等待同几何尺寸且包含本轮障碍的全局代价地图'
        return ''

    def on_goal(self, message):
        if self.active_request is not None:
            self.get_logger().warning('正在规划，忽略新目标；完成后再点击')
            return
        reason = self.readiness_issue()
        if reason:
            self.set_status(reason)
            return
        goal = copy.deepcopy(message)
        if goal.header.frame_id != 'map':
            self.reject('第一版只接受 map 坐标系目标')
            return
        p, q = goal.pose.position, goal.pose.orientation
        values = (p.x, p.y, p.z, q.x, q.y, q.z, q.w)
        if not all(math.isfinite(v) for v in values):
            self.reject('目标位姿含非有限数值')
            return
        norm = math.sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w)
        if norm < 1e-9 or abs(q.x) > 1e-6 or abs(q.y) > 1e-6 or abs(p.z) > 1e-6:
            self.reject('目标须为平面位姿，且四元数不能全为零')
            return
        q.z, q.w = q.z / norm, q.w / norm
        issue = point_issue(self.map, p.x, p.y)
        if issue:
            self.reject('目标' + issue)
            return

        goal.header.stamp = self.get_clock().now().to_msg()
        request = ComputePathToPose.Goal()
        request.goal = goal
        request.planner_id = 'GridBased'
        request.use_start = False  # Nav2 从 TF 查询当前起点。
        self.request_number += 1
        number = self.request_number
        self.active_request = number
        self.request_time = time.monotonic()
        self.clear_path()
        self.set_status(f'PLANNING: 请求 {number}，目标 ({p.x:.3f}, {p.y:.3f}) m')
        try:
            future = self.planner.send_goal_async(request)
            future.add_done_callback(lambda value: self.on_goal_response(value, number))
        except Exception as error:
            self.fail_request(number, str(error))

    def reject(self, reason):
        self.clear_path()
        self.set_status('REJECTED: ' + reason)

    def fail_request(self, number, reason):
        if number != self.active_request:
            return
        self.active_request = None
        self.goal_handle = None
        self.clear_path()
        self.set_status('FAILED: ' + reason)

    def on_goal_response(self, future, number):
        try:
            handle = future.result()
        except Exception as error:
            self.fail_request(number, str(error))
            return
        # 超时/地图失效后才收到受理回复，也必须取消，不能让它恢复本地任务。
        if number != self.active_request:
            if handle.accepted:
                handle.cancel_goal_async()
            return
        if not handle.accepted:
            self.fail_request(number, 'Nav2 拒绝规划请求')
            return
        self.goal_handle = handle
        result_future = handle.get_result_async()
        result_future.add_done_callback(lambda value: self.on_result(value, number))

    def on_result(self, future, number):
        if number != self.active_request:
            return
        try:
            wrapped = future.result()
            result = wrapped.result
            if wrapped.status != GoalStatus.STATUS_SUCCEEDED or result.error_code != 0:
                self.fail_request(number, f'Nav2 状态={wrapped.status}，'
                                  f'错误码={result.error_code}，{result.error_msg}')
                return
            path = result.path
            if path.header.frame_id != 'map' or not path.poses:
                self.fail_request(number, 'Nav2 返回空路径或错误坐标系')
                return
            for pose in path.poses:
                if pose.header.frame_id not in ('', 'map'):
                    raise ValueError('路径点坐标系不是 map')
                p = pose.pose.position
                if not all(math.isfinite(v) for v in (p.x, p.y, p.z)):
                    raise ValueError('路径含非法坐标')
            issue = self.readiness_issue()
            if issue:
                self.fail_request(number, '返回结果时条件失效：' + issue)
                return
        except Exception as error:
            self.fail_request(number, str(error))
            return
        self.active_request = None
        self.goal_handle = None
        self.path_pub.publish(path)
        self.has_path = True
        seconds = result.planning_time.sec + result.planning_time.nanosec * 1e-9
        self.set_status(f'SUCCEEDED: {len(path.poses)} 个路径点，规划耗时 {seconds:.3f} s；'
                        '仅显示路径，不执行电机动作')


def main(args=None):
    rclpy.init(args=args)
    node = NavigationManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.goal_handle is not None:
            node.goal_handle.cancel_goal_async()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
