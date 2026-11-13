"""Run with unittest or pytest; no ROS installation required."""

import importlib.util
import math
from pathlib import Path
import random
import sys
from types import SimpleNamespace as Obj
import unittest

CORE = Path(__file__).resolve().parents[1] / 'transport_navigation' / 'path_processing.py'
SPEC = importlib.util.spec_from_file_location('path_processing_under_test', CORE)
core = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = core
SPEC.loader.exec_module(core)


def make_path(points, yaw=0.0, frame='map'):
    poses = []
    for x, y in points:
        poses.append(Obj(header=Obj(frame_id=frame), pose=Obj(
            position=Obj(x=x, y=y, z=0.0),
            orientation=Obj(x=0.0, y=0.0, z=math.sin(yaw / 2), w=math.cos(yaw / 2)))))
    return Obj(header=Obj(frame_id=frame), poses=poses)


class PathProcessingTests(unittest.TestCase):
    def test_merge_same_direction_and_keep_corners(self):
        route = core.process_points([(0, 0), (0, 0), (1, 0), (2, 0), (2, 1), (3, 1)], 0)
        self.assertEqual(route.points, ((0, 0), (2, 0), (2, 1), (3, 1)))
        self.assertEqual([s.axis for s in route.segments], [0, 1, 0])
        self.assertEqual([s.length for s in route.segments], [2, 1, 1])
        self.assertEqual(route.total_length, 4)
        self.assertFalse(route.segments[0].has_previous_turn)
        self.assertAlmostEqual(route.segments[1].turn_from_previous, math.pi / 2)
        self.assertAlmostEqual(route.segments[2].turn_from_previous, -math.pi / 2)

    def test_keep_u_turn(self):
        route = core.process_points([(0, 0), (2, 0), (1, 0)], 0)
        self.assertEqual(len(route.points), 3)
        self.assertEqual(route.total_length, 3)
        self.assertAlmostEqual(abs(route.segments[1].turn_from_previous), math.pi)

    def test_all_axes(self):
        route = core.process_points([(0, 0), (1, 0), (1, 1), (0, 1), (0, 0)], 0)
        self.assertEqual([s.axis for s in route.segments], [0, 1, 2, 3])

    def test_reject_any_diagonal_without_snapping(self):
        for end in [(1, 1), (1, 1e-14)]:
            with self.subTest(end=end), self.assertRaises(core.InvalidPath):
                core.process_points([(0, 0), end], 0)

    def test_preserve_short_edge(self):
        route = core.process_points([(0, 0), (1e-12, 0), (1e-12, 1)], 0)
        self.assertEqual(len(route.segments), 2)
        self.assertEqual(route.segments[0].length, 1e-12)

    def test_empty_invalid(self):
        with self.assertRaises(core.InvalidPath):
            core.process_points([], 0)

    def test_single_point_and_repeated_points(self):
        route = core.process_points([(1, 2), (1, 2)], math.pi / 2)
        self.assertEqual(route.points, ((1, 2),))
        self.assertEqual(route.segments, ())
        self.assertEqual(route.total_length, 0)
        self.assertAlmostEqual(route.goal_yaw, math.pi / 2)

    def test_bad_numbers(self):
        for bad in [float('nan'), float('inf'), -float('inf')]:
            with self.subTest(bad=bad):
                with self.assertRaises(core.InvalidPath):
                    core.process_points([(bad, 0)], 0)
                with self.assertRaises(core.InvalidPath):
                    core.process_points([(0, 0)], bad)

    def test_overflow(self):
        for points in [[(-1e308, 0), (1e308, 0)],
                       [(0, 0), (1e308, 0), (1e308, 1e308)]]:
            with self.assertRaises(core.InvalidPath):
                core.process_points(points, 0)

    def test_limits(self):
        with self.assertRaises(core.InvalidPath):
            core.process_points([(0, 0), (1, 0), (1, 1)], 0, max_segments=1)
        with self.assertRaises(core.InvalidPath):
            core.process_path_message(make_path([(0, 0), (1, 0)]), max_points=1)

    def test_final_duplicate_retains_goal_orientation(self):
        path = make_path([(0, 0), (1, 0), (1, 0)])
        path.poses[-1].pose.orientation.z = 2.0
        path.poses[-1].pose.orientation.w = 2.0
        route = core.process_path_message(path)
        self.assertEqual(len(route.points), 2)
        self.assertAlmostEqual(route.goal_yaw, math.pi / 2)
        self.assertEqual(route.segments[0].heading, 0.0)

    def test_frame_validation(self):
        path = make_path([(0, 0)], frame='odom')
        with self.assertRaises(core.InvalidPath):
            core.process_path_message(path)
        path = make_path([(0, 0)])
        path.poses[0].header.frame_id = 'odom'
        with self.assertRaises(core.InvalidPath):
            core.process_path_message(path)
        path.poses[0].header.frame_id = ''
        self.assertEqual(len(core.process_path_message(path).points), 1)

    def test_quaternion_and_planarity(self):
        for field, value in [('w', 0.0), ('x', 0.5), ('w', float('nan'))]:
            path = make_path([(0, 0)])
            setattr(path.poses[0].pose.orientation, field, value)
            with self.subTest(field=field, value=value), self.assertRaises(core.InvalidPath):
                core.process_path_message(path)
        path = make_path([(0, 0)])
        path.poses[0].pose.position.z = 0.1
        with self.assertRaises(core.InvalidPath):
            core.process_path_message(path)

    def test_negative_coordinates_are_not_grid_indices(self):
        route = core.process_points([(-0.15, -0.2), (1.25, -0.2)], 0)
        self.assertAlmostEqual(route.total_length, 1.4)
        self.assertEqual(route.points[0], (-0.15, -0.2))

    def test_random_walk_preserves_every_directed_unit_edge(self):
        rng = random.Random(20261111)

        def expand(points):
            result = []
            for (x, y), (tx, ty) in zip(points, points[1:]):
                dx = 0 if x == tx else (1 if tx > x else -1)
                dy = 0 if y == ty else (1 if ty > y else -1)
                while (x, y) != (tx, ty):
                    nx, ny = x + dx, y + dy
                    result.append(((x, y), (nx, ny)))
                    x, y = nx, ny
            return result

        for _ in range(100):
            points = [(0, 0)]
            for _ in range(100):
                dx, dy = rng.choice([(1, 0), (-1, 0), (0, 1), (0, -1), (0, 0)])
                points.append((points[-1][0] + dx, points[-1][1] + dy))
            route = core.process_points(points, 0)
            self.assertEqual(expand(points), expand(route.points))
            self.assertEqual(route.total_length, len(expand(points)))


if __name__ == '__main__':
    unittest.main()
