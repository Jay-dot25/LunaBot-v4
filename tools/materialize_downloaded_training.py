"""Build D:-resident canonical training additions; preserve old held-out scenes.

COCO/YOLO polygon backgrounds and LunarSeg background remain ignore. AI4Mars
NAV training labels only are used (never geology IDs or expert test masks).
"""
import argparse
import csv
import hashlib
import json
import re
import zipfile
from collections import Counter, defaultdict
from collections import deque
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
from threading import Lock, Semaphore

import numpy as np
from PIL import Image, ImageDraw

FIELDS = ['image', 'mask', 'source_id', 'terrain_id', 'split']


def indexed(path):
    with Image.open(path) as im:
        if im.mode not in ('L', 'P'):
            raise ValueError(f'Non-indexed mask: {path}')
        return np.array(im)


def remap(mask, mapping):
    unknown = set(np.flatnonzero(np.bincount(mask.ravel(), minlength=256)).tolist()) - set(mapping)
    if unknown:
        raise ValueError(f'Unmapped source labels: {unknown}')
    table = np.full(256, 255, np.uint8)
    for old, new in mapping.items():
        table[old] = new
    return table[mask]


def polygon_mask(size, annotations):
    canvas = Image.new('L', size, 255)
    draw = ImageDraw.Draw(canvas)
    # Crater wins overlapping rock only according to the project's precedence.
    for target, polygons in sorted(annotations, key=lambda x: x[0] == 3):
        for polygon in polygons:
            if len(polygon) < 6 or len(polygon) % 2:
                raise ValueError('Invalid segmentation polygon')
            draw.polygon(list(zip(polygon[::2], polygon[1::2])), fill=target)
    return np.asarray(canvas)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--repo', type=Path, default=Path('/mnt/d/lunabot-v4'))
    p.add_argument('--data', type=Path, default=Path('/mnt/d/data/lunabot-datasets'))
    p.add_argument('--size', type=int, default=384)
    p.add_argument('--recover-completed-polygons', action='store_true',
                   help='reuse the already completed COCO/YOLO stage after a later-source failure')
    a = p.parse_args()
    a.output.mkdir(parents=True, exist_ok=True)
    pairs = a.output / 'pairs'
    pairs.mkdir(exist_ok=True)
    rows, lineage, rejected = [], [], Counter()
    seen = {}
    counts = Counter()
    pixels = np.zeros(5, np.int64)
    quarantined = []
    lock = Lock()
    raw_slots = Semaphore(2)
    pool = ThreadPoolExecutor(max_workers=8)
    pending = deque()

    def convert(ip, mask, source, scene, original_mask):
        key = hashlib.sha256((source + str(ip)).encode()).hexdigest()
        image_out, mask_out = pairs / (key + '.png'), pairs / (key + '-mask.png')
        cached = False
        if image_out.is_file() and mask_out.is_file():
            try:
                with Image.open(image_out) as im:
                    resized_image = im.convert('RGB')
                resized_mask = indexed(mask_out)
                cached = (resized_image.size == (a.size, a.size) and resized_mask.shape == (a.size, a.size)
                          and not set(np.unique(resized_mask).tolist()) - {0, 1, 2, 3, 4, 255})
            except (OSError, ValueError):
                cached = False
        if not cached:
            if mask is None:
                with lock:
                    rejected['completed_polygon_stage_duplicate_or_unlabelled'] += 1
                return
            with raw_slots:
                try:
                    source_mask = mask() if callable(mask) else mask
                except ValueError as exc:
                    if not str(exc).startswith('Unmapped source labels:'):
                        raise
                    with lock:
                        quarantined.append(dict(image=str(ip), annotation=str(original_mask), reason=str(exc)))
                        rejected['undefined_source_label_quarantined'] += 1
                    return
                resized_mask = np.array(Image.fromarray(source_mask).resize((a.size, a.size), Image.Resampling.NEAREST))
                if not (resized_mask != 255).any():
                    with lock:
                        rejected['no_labeled_pixels'] += 1
                    return
                with Image.open(ip) as im:
                    if im.size != (source_mask.shape[1], source_mask.shape[0]):
                        raise ValueError(f'Image-mask size mismatch: {ip}')
                    resized_image = im.convert('RGB').resize((a.size, a.size), Image.Resampling.BILINEAR)
        if not (resized_mask != 255).any():
            with lock:
                rejected['no_labeled_pixels'] += 1
            return
        fingerprint = hashlib.sha256(resized_image.resize((256, 256), Image.Resampling.BILINEAR).tobytes()).hexdigest()
        with lock:
            if fingerprint in seen:
                rejected['duplicate_or_heldout_image'] += 1
                return
            seen[fingerprint] = 'train'
        if not cached:
            resized_image.save(image_out, compress_level=1)
            Image.fromarray(resized_mask).save(mask_out, compress_level=1)
        values, ns = np.unique(resized_mask, return_counts=True)
        with lock:
            rows.append(dict(image=str(image_out), mask=str(mask_out), source_id=source, terrain_id=scene, split='train'))
            lineage.append(dict(image=str(ip), annotation=str(original_mask), output_key=key,
                                rgb256_sha256=fingerprint, fingerprint_basis='canonical RGB cache resized to 256'))
            counts[source] += 1
            for v, n in zip(values, ns):
                if v < 5:
                    pixels[v] += n
            if sum(counts.values()) % 500 == 0:
                print('Converted', dict(counts), flush=True)

    def add(*values):
        pending.append(pool.submit(convert, *values))
        if len(pending) >= 16:
            pending.popleft().result()

    # Rebase existing reviewed pairs; evaluation file content and splits stay fixed.
    old = a.repo / '.build/p502-cache-reviewed-v2/manifest.csv'
    with old.open() as f:
        old_rows = list(csv.DictReader(f))
    for row in sorted(old_rows, key=lambda r: r['split'] == 'train'):
        for field in ('image', 'mask'):
            value = row[field].replace('/home/jay/lunabot-v4/', str(a.repo) + '/')
            path = Path(value)
            if not path.is_absolute():
                path = old.parent / path
            row[field] = str(path)
        # The separate parallel corpus audit checks every inherited and new
        # image together before packaging. Avoid repeatedly decoding the
        # unchanged inherited corpus during a later-source recovery.
        rows.append({k: row[k] for k in FIELDS})
    print('Inherited reviewed corpus:', len(rows), flush=True)

    coco = a.data / 'coco/Dataset_coco/train'
    doc = json.loads((coco / '_annotations.coco.json').read_text())
    cats = {c['id']: c['name'] for c in doc['categories']}
    grouped = defaultdict(list)
    for ann in doc['annotations']:
        name = cats[ann['category_id']]
        if name not in ('rock', 'crater'):
            raise ValueError(f'Unknown COCO class: {name}')
        seg = ann['segmentation']
        if not isinstance(seg, list):
            raise ValueError('RLE annotations require an explicit decoder')
        grouped[ann['image_id']].append((2 if name == 'rock' else 3, seg))
    for im in doc['images']:
        mask = None if a.recover_completed_polygons else polygon_mask((im['width'], im['height']), grouped[im['id']])
        add(coco / im['file_name'], mask, 'roboflow-combined-polygons', 'download-train', coco / '_annotations.coco.json')
    # Preserve COCO's deterministic priority over overlapping YOLO v2 labels.
    while pending:
        pending.popleft().result()

    # YOLO v2 can contain additional train images. RGB fingerprinting deduplicates
    # overlapping exports against COCO and all inherited evaluation images.
    yolo = a.data / 'yolo/Dataset_yolo/train'
    for ip in sorted((yolo / 'images').glob('*')):
        label = yolo / 'labels' / (ip.stem + '.txt')
        if a.recover_completed_polygons:
            add(ip, None, 'roboflow-combined-polygons', 'download-train', label)
            continue
        if not label.is_file():
            rejected['yolo_missing_label'] += 1
            continue
        with Image.open(ip) as im:
            w, h = im.size
        anns = []
        for line in label.read_text().splitlines():
            values = [float(v) for v in line.split()]
            if not values:
                continue
            cls, xy = int(values[0]), values[1:]
            if cls not in (0, 1) or len(xy) < 6 or len(xy) % 2:
                raise ValueError(f'Expected YOLO segmentation polygon: {label}')
            coords = [v * (w if i % 2 == 0 else h) for i, v in enumerate(xy)]
            anns.append((3 if cls == 0 else 2, [coords]))
        add(ip, polygon_mask((w, h), anns), 'roboflow-combined-polygons', 'download-train', label)

    # LunarSeg: source order background, negative obstacle, positive obstacle.
    # Background includes unclassified surfaces: never infer regolith/shadow.
    lunar = a.data / 'lunarseg/LunarSeg'
    for name in (lunar / 'train.txt').read_text().splitlines():
        name = name.strip().removesuffix('.png')
        if not name:
            continue
        mp = lunar / 'label' / (name + '.png')
        add(lunar / 'image' / (name + '.png'), lambda mp=mp: remap(indexed(mp), {0: 255, 1: 3, 2: 2, 255: 255}),
            'lunarseg-obstacles', name.split('_')[0], mp)

    staging = json.loads((a.data / 'staging.json').read_text())
    ai = Path(staging['ai4mars']['extracted_root']) / 'ai4mars-dataset-merged-0.6'
    # Curiosity NAV has cleaned labels. MER merged-unmasked labels remain
    # staged but excluded until rover/range masking is reviewed; M2020 GEO
    # and expert test labels are not interpreted as NAV training.
    with zipfile.ZipFile(staging['ai4mars']['archive']) as archive:
        inventory = [n for n in archive.namelist() if not n.endswith('/')]
    for relative_base in ('msl/ncam',):
        base = ai / relative_base
        prefix = 'ai4mars-dataset-merged-0.6/' + relative_base + '/'
        image_index = defaultdict(list)
        for member in inventory:
            if member.startswith(prefix + 'images/') and Path(member).suffix.lower() in ('.jpg', '.jpeg'):
                ip = ai.parent / member
                image_index[ip.stem.lower()].append(ip)
        for member in sorted(n for n in inventory if n.startswith(prefix + 'labels/train/') and n.endswith('.png')):
            mp = ai.parent / member
            stem = re.sub(r'_merged\d*$', '', mp.stem, flags=re.I).lower()
            candidates = image_index.get(stem, [])
            if not candidates and re.search(r'_\d+$', stem):
                # MSL MastCam labels append an annotation identifier before
                # _merged; image basenames stop at _XXXX.
                candidates = image_index.get(stem.rsplit('_', 1)[0], [])
            if len(candidates) != 1:
                rejected['ai4mars_unpaired_or_ambiguous'] += 1
                continue
            add(candidates[0], lambda mp=mp: remap(indexed(mp), {0: 0, 1: 1, 2: 0, 3: 2, 255: 255}),
                'ai4mars-nav-training', base.relative_to(ai).as_posix(), mp)

    while pending:
        pending.popleft().result()
    pool.shutdown()
    rows.sort(key=lambda row: (row['split'], row['source_id'], row['image']))
    lineage.sort(key=lambda row: row['output_key'])
    with (a.output / 'manifest.csv').open('w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=FIELDS)
        writer.writeheader()
        writer.writerows(rows)
    frozen = a.repo / 'evidence/runs/20261107-p502/P5-02'
    sources = json.loads((frozen / 'sources-frozen.json').read_text())
    splits = json.loads((frozen / 'splits-frozen.json').read_text())
    for sid in counts:
        sources['selected_sources'].append(dict(source_id=sid, title=sid,
            status='user_supplied_training_only', limitations=['Not held-out lunar qualification evidence.']))
        if sid not in splits['train_only_sources']:
            splits['train_only_sources'].append(sid)
    for name, doc in [('sources.json', sources), ('splits.json', splits)]:
        (a.output / name).write_text(json.dumps(doc, indent=2) + '\n')
    report = dict(new_records=dict(counts), new_class_pixels=pixels.tolist(), rejected=dict(rejected),
                  inherited_manifest_sha256=hashlib.sha256(old.read_bytes()).hexdigest(),
                  records=dict(Counter(r['split'] for r in rows)), lineage=lineage,
                  quarantined_records=quarantined, recovered_completed_polygons=a.recover_completed_polygons,
                  mapping=dict(ai4mars_nav={'0': 0, '1': 1, '2': 0, '3': 2, '255': 255},
                               lunarseg={'0': 255, '1': 3, '2': 2, '255': 255}),
                  limitation='Training additions only. Existing sparse held-out support is unchanged. No qualification claim.')
    report['excluded_subsets'] = ['MSL MastCam: undefined source label 4 observed; ontology not established', 'MER merged-unmasked: rover/range masking not reviewed', 'M2020 GEO: different ontology', 'All newly downloaded validation/test splits']
    (a.output / 'materialization.json').write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps({k: v for k, v in report.items() if k != 'lineage'}, indent=2), flush=True)


if __name__ == '__main__':
    main()
