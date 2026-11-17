"""Audit canonical pair integrity, class support and exact cross-split leakage."""
import argparse
import csv
import hashlib
import json
from collections import Counter
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

import numpy as np
from PIL import Image


def check(row):
    with Image.open(row['image']) as im:
        size = im.size
        rgb = im.convert('RGB')
        digest = hashlib.sha256(rgb.resize((256, 256), Image.Resampling.BILINEAR).tobytes()).hexdigest()
    with Image.open(row['image']) as im:
        im.verify()
    with Image.open(row['mask']) as im:
        if im.mode not in ('L', 'P') or im.size != size:
            raise ValueError(f'Invalid image-mask pair: {row}')
        hist = np.bincount(np.asarray(im).ravel(), minlength=256)
        if hist[5:255].any():
            raise ValueError(f'Non-canonical mask IDs: {row}')
    return row, digest, hist[:5]


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--corpus', type=Path, required=True)
    a = p.parse_args()
    with (a.corpus / 'manifest.csv').open() as stream:
        rows = list(csv.DictReader(stream))
    hist = {s: np.zeros(5, np.int64) for s in ['train', 'validation', 'test']}
    seen, collisions, counts = {}, [], Counter()
    with ThreadPoolExecutor(max_workers=8) as pool:
        for i, (row, digest, pixels) in enumerate(pool.map(check, rows), 1):
            split = row['split']
            hist[split] += pixels
            if digest in seen and seen[digest] != split:
                collisions.append(dict(image=row['image'], splits=[seen[digest], split]))
            seen[digest] = split
            counts[(split, row['source_id'])] += 1
            if i % 2000 == 0:
                print('Audited', i, '/', len(rows), flush=True)
    missing = {s: np.flatnonzero(h == 0).tolist() for s, h in hist.items() if (h == 0).any()}
    doc = dict(result='PASS' if not missing and not collisions else 'FAIL', records=len(rows),
               configuration_sha256={name: hashlib.sha256((a.corpus / name).read_bytes()).hexdigest()
                                     for name in ['manifest.csv', 'sources.json', 'splits.json']},
               split_records=dict(Counter(r['split'] for r in rows)),
               source_records={f'{s}/{source}': n for (s, source), n in counts.items()},
               class_pixels={s: h.tolist() for s, h in hist.items()}, missing_classes=missing,
               cross_split_identical_rgb256=collisions,
               limitation='Exact image checking is not a near-duplicate or scene-independence audit.')
    (a.corpus / 'audit.json').write_text(json.dumps(doc, indent=2) + '\n')
    print(json.dumps(doc, indent=2), flush=True)
    return 0 if doc['result'] == 'PASS' else 1


if __name__ == '__main__':
    raise SystemExit(main())
