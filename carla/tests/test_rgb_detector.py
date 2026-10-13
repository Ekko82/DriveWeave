"""Offline contracts for real YOLOX RGB inference; no CARLA or GT needed."""
import importlib.util
from pathlib import Path
import unittest

import numpy as np

MODULE = Path(__file__).resolve().parents[1] / "detect_rgb_onnx.py"
spec = importlib.util.spec_from_file_location("detect_rgb_onnx", MODULE)
detector = importlib.util.module_from_spec(spec)
spec.loader.exec_module(detector)


def output_with_box(index=0, class_id=2, score=.9):
    raw = np.zeros((1, 8400, 85), dtype=np.float32)
    raw[0, index, :4] = [10, 10, np.log(5), np.log(3)]
    raw[0, index, 4] = .8
    raw[0, index, 5 + class_id] = score
    return raw


class TestRgbDetector(unittest.TestCase):
    def test_preprocessing_bgr_and_unnormalized_pad(self):
        rgb = np.full((2, 4, 3), [10, 20, 30], dtype=np.uint8)
        original = rgb.copy()
        tensor, ratio = detector.preprocess_rgb(rgb)
        self.assertEqual(tensor.shape, (3, 640, 640))
        self.assertEqual(tensor.dtype, np.float32)
        self.assertEqual(ratio, 160)
        np.testing.assert_array_equal(tensor[:, 0, 0], [30, 20, 10])
        np.testing.assert_array_equal(tensor[:, 319, 639], [30, 20, 10])
        np.testing.assert_array_equal(tensor[:, 320, 0], [114, 114, 114])
        np.testing.assert_array_equal(rgb, original)

    def test_invalid_input_has_no_silent_conversion(self):
        for image in [np.zeros((2, 2), dtype=np.uint8), np.zeros((2, 2, 4), dtype=np.uint8),
                      np.zeros((2, 2, 3), dtype=np.float32), np.zeros((0, 2, 3), dtype=np.uint8)]:
            with self.assertRaises(ValueError):
                detector.preprocess_rgb(image)

    def test_three_stride_grid_order_and_input_immutable(self):
        raw = np.zeros((1, 8400, 85), dtype=np.float32)
        original = raw.copy()
        result = detector.decode_yolox(raw)
        np.testing.assert_array_equal(result[79, :4], [632, 0, 8, 8])
        np.testing.assert_array_equal(result[80, :4], [0, 8, 8, 8])
        np.testing.assert_array_equal(result[6400, :4], [0, 0, 16, 16])
        np.testing.assert_array_equal(result[8000, :4], [0, 0, 32, 32])
        np.testing.assert_array_equal(raw, original)

    def test_objectness_times_class_probability_and_scale(self):
        result = detector.postprocess(output_with_box(), .5, (720, 1280), .3)
        self.assertEqual(len(result), 1)
        self.assertEqual(result[0]["class_id"], 2)
        self.assertEqual(result[0]["class_name"], "car")
        self.assertAlmostEqual(result[0]["score"], .72, places=6)
        np.testing.assert_allclose(result[0]["bbox_xyxy"], [120, 136, 200, 184], atol=1e-4)
        self.assertEqual(detector.postprocess(output_with_box(score=.2), 1, (640, 640), .3), [])

    def test_class_agnostic_nms_and_nonroad_argmax(self):
        raw = output_with_box()
        raw[0, 1, :4] = [9, 10, np.log(5), np.log(3)]
        raw[0, 1, 4] = .9
        raw[0, 1, 5 + 7] = .9
        result = detector.postprocess(raw, 1, (640, 640), .3)
        self.assertEqual(len(result), 1)
        self.assertEqual(result[0]["class_name"], "truck")
        raw = output_with_box()
        raw[0, 0, 5 + 56] = .95  # chair wins argmax over car; never relabel as road class
        self.assertEqual(detector.postprocess(raw, 1, (640, 640), .3), [])

    def test_all_six_road_classes_and_no_gt_namespace(self):
        for class_id, name in detector.ROAD_CLASSES.items():
            result = detector.postprocess(output_with_box(class_id=class_id), 1, (640, 640), .3)
            self.assertEqual(result[0]["class_id"], class_id)
            self.assertEqual(result[0]["class_name"], name)
        self.assertNotIn(14, detector.ROAD_CLASSES)

    def test_offscreen_boxes_clipped_or_rejected(self):
        raw = output_with_box()
        raw[0, 0, :4] = [0, 0, np.log(5), np.log(3)]
        result = detector.postprocess(raw, 1, (640, 640), .3)
        np.testing.assert_allclose(result[0]["bbox_xyxy"], [0, 0, 20, 12], atol=1e-4)
        raw[0, 0, :2] = [-100, -100]
        self.assertEqual(detector.postprocess(raw, 1, (640, 640), .3), [])

    def test_model_output_shape_and_nonfinite_are_hard_failures(self):
        for raw in [np.zeros((1, 8400, 84), dtype=np.float32),
                    np.full((1, 8400, 85), np.nan, dtype=np.float32)]:
            with self.assertRaises(ValueError):
                detector.decode_yolox(raw)


if __name__ == "__main__":
    unittest.main()
