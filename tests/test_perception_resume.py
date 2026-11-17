"""Exercise interruption recovery against an uninterrupted deterministic run."""
import csv
import importlib.util
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

import numpy as np
from PIL import Image

ROOT = Path(__file__).resolve().parents[1]


@unittest.skipUnless(importlib.util.find_spec('torch'), 'requires training runtime')
class ResumeTests(unittest.TestCase):
    def test_resume_matches_uninterrupted_and_rejects_changed_gates(self):
        import torch
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            rows = []
            for split, terrain in [('train', 'terrain01'), ('validation', 'terrain09'), ('test', 'terrain10')]:
                image, mask = root / (split+'.png'), root / (split+'-mask.png')
                labels = np.indices((32, 32)).sum(axis=0).astype(np.uint8) % 5
                Image.fromarray(np.stack([labels*40]*3, axis=-1)).save(image)
                Image.fromarray(labels).save(mask)
                rows.append(dict(image=str(image), mask=str(mask), source_id='polar-sim-labels', terrain_id=terrain, split=split))
            manifest = root / 'manifest.csv'
            with manifest.open('w', newline='') as stream:
                writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
                writer.writeheader()
                writer.writerows(rows)
            gates = root / 'gates.json'
            gates.write_text(json.dumps(dict(schema_version=1, gate_id='synthetic-resume-test', minimum_miou=0,
                minimum_per_class_iou={n: 0 for n in ['regolith','bedrock','rock','crater','shadow']})))
            common = [sys.executable, str(ROOT/'tools/train_perception.py'), '--manifest', str(manifest),
                      '--gates', str(gates), '--image-size', '32', '--batch-size', '1', '--cpu-threads', '1', '--device', 'cpu']
            def run(output, epochs, resume=False, skip_test=False):
                command = common + ['--output', str(output), '--epochs', str(epochs)]
                if resume:
                    command += ['--resume', str(output/'last.pt')]
                if skip_test:
                    command += ['--skip-test']
                return subprocess.run(command, capture_output=True, text=True)
            full, recovered = root/'full', root/'recovered'
            for output, epochs, resume in [(full, 2, False), (recovered, 1, False), (recovered, 2, True)]:
                result = run(output, epochs, resume)
                self.assertEqual(0, result.returncode, result.stdout+result.stderr)
            a = torch.load(full/'last.pt', weights_only=True)
            b = torch.load(recovered/'last.pt', weights_only=True)
            self.assertEqual(a['history'], b['history'])
            for key, tensor in a['state_dict'].items():
                self.assertTrue(torch.equal(tensor, b['state_dict'][key]), key)
            gates.write_text(gates.read_text()+'\n')
            result = run(recovered, 3, True)
            self.assertNotEqual(0, result.returncode)
            self.assertIn('configuration differ', result.stderr)
            # The preflight can hash this file, but prediction would require
            # decoding it and fail. Development must never iterate test images.
            (root/'test.png').write_bytes(b'held-out test must not be decoded')
            development = root/'development'
            result = run(development, 1, skip_test=True)
            self.assertEqual(0, result.returncode, result.stdout+result.stderr)
            self.assertFalse((development/'metrics.json').exists())
            self.assertFalse(json.loads((development/'validation-metrics.json').read_text())['test_predictions_inspected'])


if __name__ == '__main__':
    unittest.main()
