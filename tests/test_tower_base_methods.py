import unittest

import numpy as np

from jenga import base, geometry as geo
from jenga.methods import end_pinch, get_method, push
from jenga.tower import TowerState
from tests.helpers import make_cfg


class TestTower(unittest.TestCase):
    def setUp(self):
        self.cfg = make_cfg()
        self.t = TowerState(self.cfg)

    def test_initial_state(self):
        self.assertEqual(len(self.t.blocks), 9)
        self.assertEqual(self.t.top_layer(), 2)
        self.assertTrue(self.t.is_complete(2))

    def test_layers_alternate_and_sit_on_the_platform(self):
        L, W, H = self.t.dims
        X0 = self.t.target_pose(0, 1)
        self.assertAlmostEqual(X0[2, 3], self.cfg.PLATFORM_HEIGHT_M)
        self.assertAlmostEqual(geo.yaw_of(self.t.target_pose(0, 1)), np.pi / 2)
        self.assertAlmostEqual(geo.yaw_of(self.t.target_pose(1, 1)), 0.0)
        self.assertAlmostEqual(self.t.target_pose(1, 1)[2, 3], self.cfg.PLATFORM_HEIGHT_M + H + self.cfg.BLOCK_GAP_M)
        # slot 0 of a layer with long axis y is at negative x
        self.assertLess(self.t.target_pose(0, 0)[0, 3], self.t.target_pose(0, 2)[0, 3])

    def test_no_overhang_footprint_is_square_and_flush(self):
        self.assertAlmostEqual(geo.tower_across_m(self.cfg), 3 * self.t.dims[1] + 2 * self.cfg.BLOCK_GAP_M)
        self.assertAlmostEqual(geo.tower_footprint_m(self.cfg), max(self.t.dims[0], geo.tower_across_m(self.cfg)))

    def test_legal_pulls_exclude_top_complete_layer(self):
        self.assertEqual(sorted(self.t.pull_candidates()), [0, 1, 2, 3, 4, 5])

    def test_placement_goes_on_a_new_layer_edges_first(self):
        self.assertEqual(self.t.placement_slots(), [(3, 0), (3, 2), (3, 1)])

    def test_after_replacing_one_block(self):
        self.t.place(0, 3, 0)
        self.assertEqual(self.t.top_layer(), 3)
        self.assertFalse(self.t.is_complete(3))
        self.assertEqual(self.t.placement_slots(), [(3, 2), (3, 1)])
        # layer 3 (being built) and layer 2 (highest complete) are excluded; block 0 left layer 0
        self.assertEqual(sorted(self.t.pull_candidates()), [1, 2, 3, 4, 5])

    def test_completing_a_layer_moves_the_rule_up(self):
        for b, slot in ((0, 0), (2, 2), (3, 1)):
            self.t.place(b, 3, slot)
        self.assertTrue(self.t.is_complete(3))
        self.assertEqual(self.t.placement_slots()[0][0], 4)
        self.assertEqual(sorted(self.t.pull_candidates()), [1, 4, 5, 6, 7, 8])  # layers below 3, blocks 0,2,3 moved

    def test_loose_block_is_never_a_candidate(self):
        self.t.mark_loose(1)
        self.assertNotIn(1, self.t.pull_candidates())
        self.assertTrue(self.t.blocks[1].loose)


class TestBase(unittest.TestCase):
    def setUp(self):
        self.cfg = make_cfg()
        self.t = TowerState(self.cfg)

    def test_every_station_sees_the_tower_straight_ahead_at_the_standoff(self):
        cx, cy = self.cfg.TOWER_CENTER_XY
        for k in range(len(self.cfg.STATION_ANGLES_DEG)):
            X_WR = base.station_pose(self.cfg, k)
            c = base.to_robot_frame(X_WR, geo.make_pose(cx, cy, 0.0))
            self.assertAlmostEqual(c[0, 3], self.cfg.STATION_STANDOFF_M, places=9)
            self.assertAlmostEqual(c[1, 3], 0.0, places=9)

    def test_near_edge_stations(self):
        self.assertEqual(base.near_edge_stations(self.t, 0), [0])  # layer 0 slot 0
        self.assertEqual(base.near_edge_stations(self.t, 2), [2])  # layer 0 slot 2
        self.assertEqual(base.near_edge_stations(self.t, 3), [1])  # layer 1 slot 0
        self.assertEqual(base.near_edge_stations(self.t, 5), [3])  # layer 1 slot 2
        self.assertEqual(base.near_edge_stations(self.t, 1), [])  # middle block: no station

    def test_path_stays_outside_the_tower_and_ends_on_the_station(self):
        start = base.base_xy_yaw(self.cfg, 0)
        for k in (1, 2, 3):
            pts = base.base_path(self.cfg, start, k)
            cx, cy = self.cfg.TOWER_CENTER_XY
            radii = [np.hypot(x - cx, y - cy) for x, y, _ in pts]
            self.assertGreaterEqual(min(radii), self.cfg.STATION_STANDOFF_M - 1e-9)
            self.assertLessEqual(max(radii), self.cfg.BASE_TRAVEL_RADIUS_M + 1e-9)
            end = base.base_xy_yaw(self.cfg, k)
            self.assertAlmostEqual(pts[-1][0], end[0], places=9)
            self.assertAlmostEqual(pts[-1][1], end[1], places=9)
            self.assertAlmostEqual((pts[-1][2] - end[2] + np.pi) % (2 * np.pi) - np.pi, 0.0, places=9)

    def test_sampled_path_respects_speed(self):
        pts = base.base_path(self.cfg, base.base_xy_yaw(self.cfg, 0), 2)
        s = base.sample_path(pts, self.cfg.BASE_SPEED_MS, 0.05)
        step = self.cfg.BASE_SPEED_MS * 0.05
        d = [np.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(s[:-1], s[1:])]
        self.assertLessEqual(max(d), step * 1.05)
        self.assertEqual(s[-1], tuple(pts[-1]))

    def test_nearest_station_is_the_shortest_base_move(self):
        self.assertEqual(base.nearest_station(self.cfg, [1, 3], 0), 1)
        self.assertEqual(base.nearest_station(self.cfg, [0, 2], 3), 0)


class TestEndPinchRules(unittest.TestCase):
    def setUp(self):
        self.cfg = make_cfg()
        self.t = TowerState(self.cfg)

    def test_only_edge_blocks_can_be_pulled(self):
        pullable = [b for b in self.t.blocks if end_pinch.can_pull(self.t, b, self.cfg)]
        self.assertEqual(sorted(pullable), [0, 2, 3, 5, 6, 8])

    def test_far_slot_is_filled_from_the_opposite_station(self):
        # new layer 3 has its long axis along x, blocks side by side along y; stations 1 (+y) and 3 (-y)
        self.assertEqual(end_pinch.place_stations(self.t, 3, 0, self.cfg), [1])
        self.assertEqual(end_pinch.place_stations(self.t, 3, 2, self.cfg), [3])
        self.assertEqual(end_pinch.place_stations(self.t, 3, 1, self.cfg), [1, 3])
        self.assertEqual(end_pinch.choose_place_station(self.t, 3, 1, self.cfg, current_station=3), 3)

    def test_every_empty_slot_can_be_placed_by_end_pinch(self):
        self.assertTrue(all(end_pinch.can_place(self.t, l, s, self.cfg) for l, s in self.t.placement_slots()))

    def test_choose_station_for_a_pull(self):
        self.assertEqual(end_pinch.choose_station(self.t, 0, self.cfg, current_station=2), 0)

    def test_method_registry(self):
        self.assertIs(get_method("end_pinch"), end_pinch)
        with self.assertRaises(ValueError):
            get_method("nope")


class TestPushRules(unittest.TestCase):
    def setUp(self):
        self.cfg = make_cfg()
        self.t = TowerState(self.cfg)

    def angles(self, block_id):
        return {self.cfg.STATION_ANGLES_DEG[k] for k in base.push_stations(self.t, block_id)}

    def test_every_block_has_two_push_stations_middle_ones_too(self):
        for bid in self.t.blocks:  # includes the middle blocks 1, 4, 7
            self.assertEqual(len(base.push_stations(self.t, bid)), 2, f"block {bid}")
            self.assertTrue(push.can_pull(self.t, bid, self.cfg))

    def test_push_stations_face_the_block_ends(self):
        # layer 0: long axis along world y, so the robot faces +y or -y; layer 1: long axis along x
        for bid in (0, 1, 2, 6, 7, 8):
            self.assertEqual(self.angles(bid), {90.0, 270.0}, f"block {bid}")
        for bid in (3, 4, 5):
            self.assertEqual(self.angles(bid), {0.0, 180.0}, f"block {bid}")

    def test_a_loose_block_has_no_push_station(self):
        self.t.mark_loose(4)
        self.assertEqual(base.push_stations(self.t, 4), [])
        self.assertFalse(push.can_pull(self.t, 4, self.cfg))

    def test_push_choose_station_picks_the_nearer_end(self):
        # block 1 (layer 0) can be pushed from station 1 (90 deg) or 3 (270 deg)
        self.assertEqual(push.choose_station(self.t, 1, self.cfg, current_station=3), 3)
        self.assertEqual(push.choose_station(self.t, 1, self.cfg, current_station=1), 1)
        # block 4 (layer 1) from station 0 (0 deg) or 2 (180 deg); from station 2 it stays put
        self.assertEqual(push.choose_station(self.t, 4, self.cfg, current_station=2), 2)

    def test_pull_station_is_the_other_end(self):
        self.assertEqual(push.choose_pull_station(self.t, 1, self.cfg, push_station=1), 3)
        self.assertEqual(push.choose_pull_station(self.t, 1, self.cfg, push_station=3), 1)
        self.assertEqual(push.choose_pull_station(self.t, 4, self.cfg, push_station=0), 2)


if __name__ == "__main__":
    unittest.main()
