import importlib.util
import math
from pathlib import Path
import sys
from types import SimpleNamespace as O
import unittest

spec = importlib.util.spec_from_file_location('route_execution_tested',
    Path(__file__).resolve().parents[1] / 'transport_navigation/route_execution.py')
core = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = core
spec.loader.exec_module(core)


def point(x, y):
    return O(x=x, y=y, z=0.0)


def quaternion(yaw=0):
    return O(x=0.0, y=0.0, z=math.sin(yaw/2), w=math.cos(yaw/2))


def message():
    points = [(1, 1), (2, 1), (2, 2)]
    return O(valid=True, route_id='test', header=O(frame_id='map'),
             waypoints=[O(position=point(*p), orientation=quaternion()) for p in points],
             segments=[O(index=i, axis=i, start=point(*points[i]), end=point(*points[i+1]),
                         heading=i*math.pi/2, length=1.0) for i in range(2)],
             goal_yaw=0.0, total_length=2.0)


def grid():
    return O(header=O(frame_id='map'), info=O(width=40, height=40, resolution=0.1,
              origin=O(position=point(0, 0), orientation=quaternion())), data=[0]*1600)


class GeometryTests(unittest.TestCase):
    def test_route_and_steps(self):
        route = core.validate_route(message())
        steps = core.build_steps(route, math.pi/2)
        self.assertEqual([s.kind for s in steps], [1, 2, 1, 2, 1])
        self.assertAlmostEqual(steps[0].yaw, -math.pi/2)
        self.assertAlmostEqual(steps[2].yaw, 0)
        self.assertEqual(steps[-1].phase, 'FINAL_ROTATION')

    def test_transform_translation_and_rotation(self):
        p = core.to_map((1, 0, math.pi/2), (2, 3, math.pi/2))
        self.assertAlmostEqual(p[0], 2)
        self.assertAlmostEqual(p[1], 4)
        self.assertAlmostEqual(abs(p[2]), math.pi)

    def test_reject_diagonal(self):
        m = message()
        m.waypoints[1].position.y = 1.1
        with self.assertRaises(ValueError):
            core.validate_route(m)

    def test_reject_wrong_length_axis_and_heading(self):
        for key, value in [('length', 3), ('axis', 3), ('heading', float('nan'))]:
            m = message()
            setattr(m.segments[0], key, value)
            with self.subTest(key=key), self.assertRaises(ValueError):
                core.validate_route(m)

    def test_reject_bad_goal_orientation(self):
        m = message()
        m.goal_yaw = math.pi
        with self.assertRaises(ValueError):
            core.validate_route(m)

    def test_reject_invalid_and_wrong_frame(self):
        for field in ('valid', 'frame'):
            m = message()
            if field == 'valid':
                m.valid = False
            else:
                m.header.frame_id = 'odom'
            with self.assertRaises(ValueError):
                core.validate_route(m)

    def test_single_point_final_rotation(self):
        m = message()
        m.waypoints = m.waypoints[:1]
        m.segments, m.total_length = [], 0.0
        self.assertEqual(len(core.build_steps(core.validate_route(m), 0)), 1)

    def test_collision_free(self):
        core.collision_check(core.validate_route(message()), grid(), 0.2)

    def test_collision_and_unknown(self):
        for value in (100, -1):
            g = grid()
            g.data[10*40+15] = value
            with self.assertRaises(ValueError):
                core.collision_check(core.validate_route(message()), g, 0.2)

    def test_cell_area_not_just_cell_center(self):
        g = grid()
        g.data[11*40+15] = 100
        # Obstacle bottom y=1.1 intersects radius .11; centre y=1.15 does not.
        with self.assertRaises(ValueError):
            core.collision_check(core.validate_route(message()), g, 0.11)

    def test_boundary_and_budget(self):
        for radius, budget in [(1.0, 2000000), (0.1, 1)]:
            with self.assertRaises(ValueError):
                core.collision_check(core.validate_route(message()), grid(), radius, budget)

    def test_lower_cell_tangency_is_blocked(self):
        g = grid()
        g.info.resolution = 1.0
        g.data[1*40+3] = 100
        r = core.Route('tangent', ((3, 3), (5, 3)), (0,), (2,), 0)
        with self.assertRaises(ValueError):
            core.collision_check(r, g, 1.0)

    def test_map_digest_ignores_stamp_detects_geometry_and_cells(self):
        g = grid()
        digest = core.map_digest(g)
        g.header.stamp = 123
        self.assertEqual(core.map_digest(g), digest)
        g.data[0] = 100
        self.assertNotEqual(core.map_digest(g), digest)

    def test_corridor_error_and_overshoot(self):
        self.assertAlmostEqual(core.line_error((1.5, 1.02), (1, 1), (2, 1)), 0.02)
        self.assertAlmostEqual(core.line_error((2.1, 1), (1, 1), (2, 1)), 0.1)


if __name__ == '__main__':
    unittest.main()
