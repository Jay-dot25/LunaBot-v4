import math
import unittest

import numpy as np

from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from semantic_projection import (
    CameraIntrinsics,
    decode_depth_buffer,
    OBSTACLE,
    RigidTransform,
    TERRAIN,
    project_pixel_to_target,
    project_synced_frame,
)


IDENTITY = RigidTransform((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))


class CalibratedSemanticProjectionTests(unittest.TestCase):
    def test_depth_decoder_handles_meters_big_endian_and_row_padding(self):
        meter_data = (np.array([1.25], dtype=">f4").tobytes() + b"pad!"
                      + np.array([2.5], dtype=">f4").tobytes() + b"pad!")
        decoded_m = decode_depth_buffer(meter_data, 1, 2, 8, "32FC1", True)
        np.testing.assert_allclose(decoded_m[:, 0], [1.25, 2.5])

        millimeter_data = (np.array([1250], dtype="<u2").tobytes() + b"pad!!!"
                           + np.array([2500], dtype="<u2").tobytes() + b"pad!!!")
        decoded_mm = decode_depth_buffer(millimeter_data, 1, 2, 8, "16UC1", False)
        np.testing.assert_allclose(decoded_mm[:, 0], [1.25, 2.5])

    def test_principal_point_uses_camera_pitch_transform(self):
        intrinsics = CameraIntrinsics(640, 480, 400.0, 400.0, 319.5, 239.5)
        pitch = 0.25
        camera_to_map = RigidTransform(
            (1.0, 2.0, 0.5), (0.0, math.sin(pitch / 2), 0.0, math.cos(pitch / 2)))
        x, y, z = project_pixel_to_target(319.5, 239.5, 2.0, intrinsics, camera_to_map)
        self.assertAlmostEqual(x, 1.0 + 2.0 * math.sin(pitch), places=7)
        self.assertAlmostEqual(y, 2.0, places=7)
        self.assertAlmostEqual(z, 0.5 + 2.0 * math.cos(pitch), places=7)

    def test_camera_intrinsics_project_known_lateral_point(self):
        intrinsics = CameraIntrinsics(640, 480, 400.0, 400.0, 319.5, 239.5)
        point = project_pixel_to_target(419.5, 239.5, 2.0, intrinsics, IDENTITY)
        self.assertAlmostEqual(point[0], 0.5, places=7)
        self.assertAlmostEqual(point[1], 0.0, places=7)
        self.assertAlmostEqual(point[2], 2.0, places=7)

    def test_sync_gate_rejects_misaligned_mask_and_depth(self):
        intrinsics = CameraIntrinsics(1, 1, 1.0, 1.0, 0.0, 0.0)
        result = project_synced_frame(
            np.array([[TERRAIN]], dtype=np.uint8), np.array([[1.0]], dtype=np.float32),
            intrinsics, intrinsics, IDENTITY, IDENTITY,
            mask_stamp_ns=100_000_000, depth_stamp_ns=200_000_000,
            max_sync_skew_ns=10_000_000, resolution=0.1, origin_x=-1,
            origin_y=-1, grid_width=20, grid_height=20)
        self.assertEqual(result.status, "WAITING_FOR_SYNC")
        self.assertEqual(result.observations, {})

    def test_missing_stale_tf_fails_closed_without_observations(self):
        intrinsics = CameraIntrinsics(1, 1, 1.0, 1.0, 0.0, 0.0)
        result = project_synced_frame(
            np.array([[TERRAIN]], dtype=np.uint8), np.array([[1.0]], dtype=np.float32),
            intrinsics, intrinsics, IDENTITY, None,
            mask_stamp_ns=100, depth_stamp_ns=100, max_sync_skew_ns=1,
            resolution=0.1, origin_x=-1, origin_y=-1, grid_width=20, grid_height=20)
        self.assertEqual(result.status, "WAITING_FOR_TF")
        self.assertEqual(result.observations, {})

    def test_rgb_z_buffer_discards_depth_hidden_by_nearer_return(self):
        depth_intrinsics = CameraIntrinsics(2, 1, 4.0, 1.0, 0.0, 0.0)
        rgb_intrinsics = CameraIntrinsics(3, 1, 4.0, 1.0, 0.0, 0.0)
        rgb_from_depth = RigidTransform((1.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))
        result = project_synced_frame(
            np.array([[0, 0, TERRAIN]], dtype=np.uint8),
            np.array([[2.0, 4.0]], dtype=np.float32),
            depth_intrinsics, rgb_intrinsics, rgb_from_depth, IDENTITY,
            mask_stamp_ns=10, depth_stamp_ns=10, max_sync_skew_ns=1,
            resolution=0.25, origin_x=-0.5, origin_y=-0.5,
            grid_width=20, grid_height=10, sample_stride=1)
        near_index = 2 * 20 + 2
        far_index = 2 * 20 + 6
        self.assertEqual(result.status, "PASS")
        self.assertEqual(result.observations, {near_index: TERRAIN})
        self.assertNotIn(far_index, result.observations)
        self.assertEqual(result.occluded_pixels, 1)

    def test_invalid_depth_and_unknown_mask_do_not_create_cells(self):
        intrinsics = CameraIntrinsics(2, 1, 4.0, 1.0, 0.0, 0.0)
        result = project_synced_frame(
            np.array([[0, 255]], dtype=np.uint8), np.array([[np.nan, 0.05]], dtype=np.float32),
            intrinsics, intrinsics, IDENTITY, IDENTITY,
            mask_stamp_ns=1, depth_stamp_ns=1, max_sync_skew_ns=0,
            resolution=0.1, origin_x=-1, origin_y=-1, grid_width=20, grid_height=20)
        self.assertEqual(result.status, "NO_VALID_DEPTH")
        self.assertEqual(result.observations, {})

    def test_obstacle_evidence_dominates_terrain_in_shared_map_cell(self):
        intrinsics = CameraIntrinsics(2, 1, 4.0, 1.0, 0.0, 0.0)
        result = project_synced_frame(
            np.array([[TERRAIN, OBSTACLE]], dtype=np.uint8), np.array([[1.0, 1.01]], dtype=np.float32),
            intrinsics, intrinsics, IDENTITY, IDENTITY,
            mask_stamp_ns=1, depth_stamp_ns=1, max_sync_skew_ns=0,
            resolution=2.0, origin_x=-1, origin_y=-1, grid_width=2, grid_height=2,
            sample_stride=1)
        self.assertEqual(result.observations, {0: OBSTACLE})

    def test_projection_carries_pixel_confidence_into_aligned_cells(self):
        intrinsics = CameraIntrinsics(1, 1, 1.0, 1.0, 0.0, 0.0)
        result = project_synced_frame(
            np.array([[TERRAIN]], dtype=np.uint8), np.array([[1.0]], dtype=np.float32),
            intrinsics, intrinsics, IDENTITY, IDENTITY,
            mask_stamp_ns=1, depth_stamp_ns=1, max_sync_skew_ns=0,
            resolution=0.1, origin_x=-0.1, origin_y=-0.1,
            grid_width=4, grid_height=4, confidence_image=np.array([[0.42]], dtype=np.float32))
        self.assertAlmostEqual(result.observation_confidence[5], 0.42, places=6)
        self.assertAlmostEqual(result.observation_elevation[5], 1.0, places=6)


if __name__ == "__main__":
    unittest.main()
