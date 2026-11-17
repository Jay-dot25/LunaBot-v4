#!/usr/bin/env python3
"""Create the Kaggle notebook for the P5-02 three-class trainer."""
from __future__ import annotations

import json
from pathlib import Path


def cell(kind: str, source: str, idx: int) -> dict:
    value = {"cell_type": kind, "id": f"cell-{idx:02d}", "metadata": {},
             "source": source.splitlines(keepends=True)}
    if kind == "code":
        value.update(execution_count=None, outputs=[])
    return value


def main() -> None:
    root = Path(__file__).resolve().parents[1]
    cells = [
        cell("markdown", """# P5-02: three-class terrain segmentation

Attach the private `p502-three-class-dataset` dataset and select a Kaggle GPU.
This notebook trains regolith, rock, and shadow. Bedrock labels are merged
into rock; crater and unknown pixels are ignored by this segmentation model.
Crater risk remains a geometry-layer responsibility.

The bundle contains the full 33,599-row canonical corpus with its existing
scene-isolated splits. The three-class qualification thresholds are mIoU >=
0.50 and IoU >= 0.25 for each output class. Test scoring stays disabled until
the final training chunk.
""", 0),
        cell("code", """from pathlib import Path
import csv, json, shutil, subprocess, sys, zipfile
import torch

# Edit these controls before each Kaggle session.
RESUME = False
TARGET_EPOCH = 1   # absolute epoch number; raise this when resuming
RUN_TEST = False   # set True only for the final session ending at epoch 30
BATCH_SIZE = 8
IMAGE_SIZE = 256
WORKERS = 2
EXPORT_RUN_ZIP = True
RESTORE_RUN_FROM = None

assert torch.cuda.is_available(), 'Enable a GPU in Kaggle Notebook settings.'
print(torch.__version__, torch.cuda.get_device_name(0))
INPUT = Path('/kaggle/input')
matches = [p for p in INPUT.rglob('manifest.csv')
           if (p.parent / 'tools/train_perception.py').is_file()
           and (p.parent / 'config/gates.json').is_file()]
if len(matches) == 1:
    BUNDLE = matches[0].parent
else:
    archives = list(INPUT.rglob('p502-three-class-dataset.zip'))
    assert len(archives) == 1, f'Attach one three-class dataset ZIP: {archives}'
    BUNDLE = Path('/kaggle/working/p502-three-class-bundle')
    marker = BUNDLE / '.archive-fingerprint'
    fingerprint = f'{archives[0]}:{archives[0].stat().st_size}:{archives[0].stat().st_mtime_ns}'
    if not marker.is_file() or marker.read_text() != fingerprint:
        assert shutil.disk_usage('/kaggle/working').free > archives[0].stat().st_size * 1.1, 'Insufficient space to unpack bundle'
        BUNDLE.mkdir(parents=True, exist_ok=True)
        with zipfile.ZipFile(archives[0]) as zf:
            zf.extractall(BUNDLE)
        marker.write_text(fingerprint)

with (BUNDLE / 'manifest.csv').open(newline='') as f:
    records = list(csv.DictReader(f))
counts = {}
for row in records:
    counts[row['split']] = counts.get(row['split'], 0) + 1
assert counts == {'train': 32104, 'validation': 746, 'test': 749}, counts
print('Bundle:', BUNDLE, 'rows:', counts)
print((BUNDLE / 'config/classes.json').read_text())
print((BUNDLE / 'config/gates.json').read_text())
""", 1),
        cell("markdown", """## Train or resume

The trainer saves `last.pt` and `best.pt` after every completed epoch. Run a
short first chunk, save the Kaggle notebook version, then attach that version's
output to a copy of this notebook. Set `RESUME=True` and raise
`TARGET_EPOCH` to the absolute epoch you want to reach. Keep the dataset
version, code, gates, batch size, image size, and worker count fixed.

For example, run epoch 1 first. Then resume with `TARGET_EPOCH=10`; continue
until epoch 30. If GPU memory is insufficient, restart from the last saved
checkpoint with `BATCH_SIZE=4`. If data-loader workers fail, use `WORKERS=0`.
""", 2),
        cell("code", """RUN = Path('/kaggle/working/p502-three-class-run')
if RESUME and not (RUN / 'last.pt').is_file():
    candidates = sorted({p.parent for p in INPUT.rglob('last.pt')
                        if (p.parent / 'best.pt').is_file()
                        and (p.parent / 'run-config.json').is_file()})
    selected = Path(RESTORE_RUN_FROM) if RESTORE_RUN_FROM else (candidates[0] if len(candidates) == 1 else None)
    assert selected is not None, f'Set RESTORE_RUN_FROM to one checkpoint directory: {candidates}'
    assert not RUN.exists(), f'Run folder already exists without a last.pt: {RUN}'
    shutil.copytree(selected, RUN)
elif not RESUME:
    assert not RUN.exists(), f'RUN already exists; set RESUME=True or choose a fresh Kaggle version: {RUN}'

assert 1 <= TARGET_EPOCH <= 30
assert not RUN_TEST or TARGET_EPOCH == 30, 'Held-out scoring is reserved for the final epoch 30 session.'
assert not RUN_TEST or not (RUN / 'metrics.json').is_file(), 'This run already has held-out metrics; do not score the test set again.'
command = [sys.executable, '-u', str(BUNDLE / 'tools/train_perception.py'),
           '--manifest', str(BUNDLE / 'manifest.csv'),
           '--manifest-root', str(BUNDLE),
           '--classes', str(BUNDLE / 'config/classes.json'),
           '--sources', str(BUNDLE / 'config/sources.json'),
           '--splits', str(BUNDLE / 'config/splits.json'),
           '--gates', str(BUNDLE / 'config/gates.json'),
           '--output', str(RUN), '--epochs', str(TARGET_EPOCH),
           '--batch-size', str(BATCH_SIZE), '--image-size', str(IMAGE_SIZE),
           '--class-weighting', 'inverse_sqrt', '--device', 'cuda', '--num-workers', str(WORKERS)]
if RESUME:
    command += ['--resume', str(RUN / 'last.pt')]
if not RUN_TEST:
    command.append('--skip-test')
print('Running:', ' '.join(command), flush=True)
subprocess.run(command, check=True)

history = json.loads((RUN / 'history.json').read_text())
print('Last completed epoch:', history[-1]['epoch'], '/ 30')
if (RUN / 'validation-metrics.json').is_file():
    validation = json.loads((RUN / 'validation-metrics.json').read_text())
    print('Selected epoch:', validation['validation_metrics']['checkpoint_epoch'])
    print('Validation mIoU:', validation['validation_metrics']['miou'])
    print('Validation class IoUs:', {k:v['iou'] for k,v in validation['validation_metrics']['per_class'].items()})
if (RUN / 'metrics.json').is_file():
    metrics = json.loads((RUN / 'metrics.json').read_text())
    print('Held-out result:', metrics['qualification_gate'])

if EXPORT_RUN_ZIP:
    output = Path('/kaggle/working/p502-three-class-run-output.zip')
    with zipfile.ZipFile(output, 'w', compression=zipfile.ZIP_DEFLATED) as zf:
        for path in sorted(RUN.rglob('*')):
            if path.is_file():
                zf.write(path, Path('p502-three-class-run') / path.relative_to(RUN))
    print('Download or attach this checkpoint archive:', output)
""", 3),
        cell("markdown", """## Final held-out evaluation

Only in the final session, set `RESUME=True`, `TARGET_EPOCH=30`, and
`RUN_TEST=True`. This runs the held-out test once after training. Read
`metrics.json` in the exported checkpoint archive. A failed gate means the
model remains unqualified for robot integration.
""", 4),
    ]
    notebook = {
        "cells": cells,
        "metadata": {
            "kernelspec": {"display_name": "Python 3", "language": "python", "name": "python3"},
            "language_info": {"name": "python"},
        },
        "nbformat": 4,
        "nbformat_minor": 5,
    }
    path = root / "notebooks/p502-three-class-kaggle.ipynb"
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(notebook, indent=2) + "\n", encoding="utf-8")
    print(path)


if __name__ == "__main__":
    main()
