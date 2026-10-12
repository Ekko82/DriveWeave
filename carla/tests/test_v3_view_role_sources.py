"""Memory-only regressions for explicit v3 video roles; no capture or media writes."""
from pathlib import Path
from types import SimpleNamespace
import json
import sys
import unittest
from unittest.mock import patch

import numpy as np
from PIL import Image

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'verification'))
import audit_v3_views as auditor


class NativeViewRoleSourcesTests(unittest.TestCase):
    def setUp(self):
        self.run = Path('virtual-native-run')
        self.processed = Path('virtual-derived-run')
        self.folders = auditor.source_folders(self.run, self.processed)
        self.colors = {
            'chase': (210, 20, 30),
            'front': (20, 190, 40),
            'rear': (190, 30, 200),
            'image-processing': (30, 50, 220),
            'lidar-processing': (220, 180, 20),
        }
        self.pictures = {
            folder: Image.new('RGB', (12, 8), self.colors[role])
            for role, folder in self.folders.items()
        }
        self.opened = []

    def tearDown(self):
        for picture in self.pictures.values():
            picture.close()

    def fake_open(self, path):
        path = Path(path)
        self.opened.append(path)
        return self.pictures[path.parent].copy()

    def reference(self, role, frame=1042, folders=None):
        with patch.object(auditor.Image, 'open', side_effect=self.fake_open):
            return auditor.reference_frame(role, frame, self.folders if folders is None else folders)

    def assert_color(self, pixels, color):
        self.assertEqual(pixels.shape, (8, 12, 3))
        self.assertTrue(np.all(pixels == np.asarray(color, dtype=np.uint8)))

    def test_folder_mapping_selects_actual_camera_and_derived_sources(self):
        self.assertEqual(self.folders, {
            'chase': self.run / 'chase',
            'front': self.run / 'rgb',
            'rear': self.run / 'rear',
            'image-processing': self.processed / 'image-processed',
            'lidar-processing': self.processed / 'lidar-3d',
        })

    def test_rear_uses_native_rear_instead_of_a_legacy_third_output(self):
        self.assert_color(self.reference('rear'), self.colors['rear'])
        self.assertEqual(self.opened, [self.run / 'rear' / '00001042.png'])

    def test_front_uses_native_rgb_camera(self):
        self.assert_color(self.reference('front'), self.colors['front'])
        self.assertEqual(self.opened, [self.run / 'rgb' / '00001042.png'])

    def test_image_processing_uses_derived_image_processed_folder(self):
        self.assert_color(self.reference('image-processing'), self.colors['image-processing'])
        self.assertEqual(self.opened, [self.processed / 'image-processed' / '00001042.png'])

    def test_lidar_processing_uses_derived_lidar_3d_folder(self):
        self.assert_color(self.reference('lidar-processing'), self.colors['lidar-processing'])
        self.assertEqual(self.opened, [self.processed / 'lidar-3d' / '00001042.png'])

    def test_role_resolution_ignores_mapping_order(self):
        reversed_folders = dict(reversed(list(self.folders.items())))
        for role in ['lidar-processing', 'rear', 'chase', 'image-processing', 'front']:
            with self.subTest(role=role):
                self.assert_color(self.reference(role, folders=reversed_folders), self.colors[role])

    def test_filename_uses_exact_native_frame_id_without_index_offset(self):
        self.reference('rear', frame=17)
        self.reference('rear', frame=1105570)
        self.assertEqual(self.opened, [
            self.run / 'rear' / '00000017.png',
            self.run / 'rear' / '01105570.png',
        ])

    def test_camera_reference_converts_pil_rgba_to_rgb(self):
        folder = self.folders['rear']
        self.pictures[folder].close()
        self.pictures[folder] = Image.new('RGBA', (12, 8), (*self.colors['rear'], 61))
        self.assert_color(self.reference('rear'), self.colors['rear'])

    def test_sensors_montage_has_exact_four_role_quadrants_and_no_rear(self):
        pixels = self.reference('sensors')
        self.assertEqual(pixels.shape, (1080, 1920, 3))
        expected_roles = ['chase', 'front', 'image-processing', 'lidar-processing']
        for index, role in enumerate(expected_roles):
            with self.subTest(quadrant=index, role=role):
                x, y = (index % 2) * 960, (index // 2) * 540
                quadrant = pixels[y:y + 540, x:x + 960]
                self.assertTrue(np.all(quadrant == np.asarray(self.colors[role], np.uint8)))
        self.assertEqual(self.opened, [
            self.folders[role] / '00001042.png' for role in expected_roles
        ])
        self.assertNotIn(self.run / 'rear' / '00001042.png', self.opened)

    def test_unknown_roles_and_numeric_legacy_indices_fail_before_image_read(self):
        for role in ['legacy-third-output', 'native-rear', 'sensors ', 0, 2, None]:
            with self.subTest(role=role):
                with patch.object(auditor.Image, 'open') as opened:
                    with self.assertRaisesRegex(ValueError, 'Unsupported explicit native v3 media role'):
                        auditor.reference_frame(role, 1042, self.folders)
                    opened.assert_not_called()

    def test_missing_montage_role_fails_without_substituting_another_view(self):
        folders = dict(self.folders)
        del folders['image-processing']
        with self.assertRaises(KeyError):
            self.reference('sensors', folders=folders)
        self.assertEqual(self.opened, [
            self.run / 'chase' / '00001042.png',
            self.run / 'rgb' / '00001042.png',
        ])

    def test_manifest_unknown_role_is_rejected_before_any_video_decode(self):
        """Exercise the public main gate, not just the reference helper."""
        ids = [101, 102, 103, 104, 105]
        manifest_path = Path('virtual-media-manifest.json')
        report_path = Path('virtual-audit-report.json')
        hashes = {
            self.run / 'manifest.json': 'native-manifest-hash',
            self.run / 'frames.jsonl': 'native-frames-hash',
            self.processed / 'processing-manifest.json': 'derived-manifest-hash',
        }
        raw = {'status': 'complete', 'native_cleanup_complete': True,
               'synthetic_sensor_fallback': False, 'frames_captured': len(ids), 'fps': 20}
        derived = {'native_frame_ids': ids, 'generated_points': False,
                   'source_manifest_sha256': hashes[self.run / 'manifest.json'],
                   'source_frames_jsonl_sha256': hashes[self.run / 'frames.jsonl']}
        spec = {
            'source_run': str(self.run), 'processed_run': str(self.processed),
            'native_frame_ids': ids,
            'capture_manifest_sha256': hashes[self.run / 'manifest.json'],
            'source_frames_sha256': hashes[self.run / 'frames.jsonl'],
            'processing_manifest_sha256': hashes[self.processed / 'processing-manifest.json'],
            'added_text': False, 'playback_speed': 1, 'interpolated_frames': False,
            'raw_sources_modified': False,
            # Position two deliberately has a legacy role; explicit membership must reject it.
            'outputs': [{'role': role} for role in [
                'chase', 'front', 'legacy-third-output',
                'image-processing', 'lidar-processing', 'sensors']],
        }
        texts = {
            manifest_path: json.dumps(spec),
            self.run / 'manifest.json': json.dumps(raw),
            self.run / 'frames.jsonl': '\n'.join(json.dumps({'frame': frame}) for frame in ids),
            self.processed / 'processing-manifest.json': json.dumps(derived),
        }
        saved = []

        def read_text(path, *args, **kwargs):
            return texts[path]

        def write_text(path, text, *args, **kwargs):
            saved.append((path, text))
            return len(text)

        with patch.object(auditor.argparse.ArgumentParser, 'parse_args',
                          return_value=SimpleNamespace(manifest=manifest_path, out=report_path)), \
                patch.object(Path, 'exists', return_value=False), \
                patch.object(Path, 'read_text', new=read_text), \
                patch.object(Path, 'write_text', new=write_text), \
                patch.object(Path, 'mkdir'), \
                patch.object(auditor.common, 'evidence_hash', side_effect=lambda path: hashes[path]), \
                patch.object(auditor.common, 'video_checks') as video_checks, \
                patch.object(auditor.imageio_ffmpeg, 'read_frames') as decoder:
            self.assertEqual(auditor.main(), 2)
            decoder.assert_not_called()
            video_checks.assert_not_called()
        self.assertEqual(len(saved), 1)
        report = json.loads(saved[0][1])
        self.assertEqual(report['overall_status'], 'fail')
        self.assertEqual(report['failed_checks'], ['six_unique_explicit_roles'])


if __name__ == '__main__':
    unittest.main()
