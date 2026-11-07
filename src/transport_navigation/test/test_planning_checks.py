"""纯 Python 地图边界和首次加载检查，不需要 ROS 安装。"""

import copy
import importlib.util
from pathlib import Path
from types import SimpleNamespace as Obj
import unittest


source = Path(__file__).parents[1] / 'transport_navigation' / 'planning_checks.py'
spec = importlib.util.spec_from_file_location('planning_checks', source)
checks = importlib.util.module_from_spec(spec)
spec.loader.exec_module(checks)


def make_grid():
    return Obj(
        header=Obj(frame_id='map', stamp=0),
        info=Obj(width=3, height=2, resolution=0.1,
                 origin=Obj(position=Obj(x=-0.1, y=-0.1, z=0.0),
                            orientation=Obj(x=0.0, y=0.0, z=0.0, w=1.0))),
        data=[0, 100, -1, 0, 0, 0],
    )


class PlanningChecksTest(unittest.TestCase):
    def test_negative_origin_and_floor(self):
        grid = make_grid()
        self.assertEqual(checks.point_issue(grid, -0.09, -0.09), '')
        self.assertIn('超出', checks.point_issue(grid, -0.10001, -0.09))

    def test_occupied_unknown_and_upper_boundary(self):
        grid = make_grid()
        self.assertIn('障碍', checks.point_issue(grid, 0.01, -0.09))
        self.assertIn('未知', checks.point_issue(grid, 0.11, -0.09))
        self.assertIn('超出', checks.point_issue(grid, 0.21, 0.0))
        self.assertIn('超出', checks.point_issue(grid, 0.0, 0.10))
        self.assertIn('非有限', checks.point_issue(grid, float('nan'), 0))

    def test_bad_map_rejected(self):
        grid = make_grid()
        self.assertTrue(checks.valid_map(grid))
        grid.info.resolution = 0
        self.assertFalse(checks.valid_map(grid))
        grid = make_grid()
        grid.info.origin.orientation.z = 0.5
        self.assertFalse(checks.valid_map(grid))

    def test_digest_ignores_stamp_but_detects_content(self):
        grid = make_grid()
        other = copy.deepcopy(grid)
        other.header.stamp = 42
        self.assertEqual(checks.map_digest(grid), checks.map_digest(other))
        other.data[0] = 100
        self.assertNotEqual(checks.map_digest(grid), checks.map_digest(other))

    def test_costmap_must_contain_obstacle(self):
        grid = make_grid()
        other = copy.deepcopy(grid)
        other.data = [10, 100, -1, 5, 20, 0]
        self.assertTrue(checks.costmap_contains_map(grid, other))
        other.data[1] = 0
        self.assertFalse(checks.costmap_contains_map(grid, other))

    def test_different_costmap_origin_rejected(self):
        grid = make_grid()
        other = copy.deepcopy(grid)
        other.info.origin.position.x += 0.01
        self.assertFalse(checks.costmap_contains_map(grid, other))


if __name__ == '__main__':
    unittest.main()
