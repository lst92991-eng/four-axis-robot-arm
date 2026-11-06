"""Offline regression: unstable grid detection must never resize the live viewport."""
import contextlib
import io
import json
from pathlib import Path
import sys
import tempfile
import time
import unittest
from unittest.mock import patch

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'temp' / 'yolo_live'))
import dual_preview as preview


class ViewportTest(unittest.TestCase):
    def test_inference_crop_boxes_map_to_display_without_changing_labels(self):
        detection={'bbox':[10,20,40,60],'class_id':3,'score':.9}
        result=preview.display_detections([detection],[300,70,650,600],[100,50,1000,650])
        self.assertEqual(result[0]['bbox'],[210,40,240,80])
        self.assertEqual(result[0]['class_id'],3)
        self.assertEqual(result[0]['score'],.9)
        self.assertEqual(detection['bbox'],[10,20,40,60])

    def test_grid_jitter_failure_and_recovery_keep_pixels_and_size(self):
        source = np.random.default_rng(1).integers(0, 256, (720, 1280, 3), dtype=np.uint8)
        geometries = [
            {'geometry_valid': True, 'points': [{'pixel_geometric': p} for p in [(320, 60), (900, 640)]]},
            {'geometry_valid': True, 'points': [{'pixel_geometric': p} for p in [(250, 0), (820, 600)]]},
            {'geometry_valid': False, 'points': []},
            {'geometry_valid': True, 'points': [{'pixel_geometric': p} for p in [(330, 50), (950, 660)]]},
        ]
        for roi in (None, [100, 50, 800, 600]):
            with self.subTest(roi=roi), tempfile.TemporaryDirectory() as folder:
                out = Path(folder)
                exe, reference = out / 'tool.exe', out / 'reference.yml'
                exe.touch()
                reference.touch()
                frames, captures = [], []

                class Capture:
                    error = None
                    def latest(self):
                        if len(captures) == len(geometries):
                            raise StopIteration
                        captures.append(1)
                        return source.copy(), time.monotonic(), len(captures)
                    def close(self):
                        pass

                def publish(board, jpeg, frame_id, geometry):
                    frames.append(cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR))

                argv = ['dual_preview', '--board', 'offline', '--geometry-exe', str(exe),
                        '--orientation-reference', str(reference), '--out', str(out)]
                if roi is not None:
                    argv += ['--roi', *map(str, roi)]
                health = json.dumps({'model_sha256': 'ce9c91f4668cb0ec58f836432546b7eb2aebfc675ba696ce9788968560dcbdd2'}).encode()
                with patch.object(sys, 'argv', argv), \
                     patch.object(preview.hp, 'Capture', return_value=Capture()), \
                     patch.object(preview.hp, 'request', return_value=health), \
                     patch.object(preview.hp, 'decode', return_value=([], {'npu_ms': 1})), \
                     patch.object(preview, 'inspect_geometry', side_effect=[(g, source.copy()) for g in geometries]), \
                     patch.object(preview, 'publish_pair', side_effect=publish), \
                     contextlib.redirect_stdout(io.StringIO()):
                    with self.assertRaises(StopIteration):
                        preview.main()
                width, height = (roi[2], roi[3]) if roi else (1280, 720)
                self.assertEqual(len(frames), 4)
                for image in frames:
                    self.assertEqual(image.shape, (height + 105, width * 2 + 8, 3))
                    # Exclude the changing header/footer and JPEG block boundaries.
                    np.testing.assert_array_equal(image[80:height + 60], frames[0][80:height + 60])
                records = [json.loads(line) for line in (out / 'frames.jsonl').read_text(encoding='utf-8').splitlines()]
                self.assertEqual([r['geometry_valid'] for r in records], [True, True, False, True])
                self.assertTrue(all(r['roi'] == (roi or [0, 0, 1280, 720]) for r in records))


if __name__ == '__main__':
    unittest.main()
