"""Create a versioned mask revision and measure rare-class scene coverage."""
import argparse
import csv
import json
import os
import shutil
from collections import Counter, defaultdict
from pathlib import Path

import numpy as np
from PIL import Image


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('corpus', type=Path)
    parser.add_argument('output', type=Path)
    args = parser.parse_args()
    corpus = args.corpus.resolve()
    output = args.output.resolve()
    with (corpus / 'manifest.csv').open(encoding='utf-8-sig', newline='') as stream:
        rows = list(csv.DictReader(stream))
    policy = json.loads((corpus / 'config/splits.json').read_text())
    namespaces = {entry['source_id']: entry.get('group_namespace', entry['source_id'])
                  for entry in policy['scene_splits']}
    output.mkdir(parents=True)
    (output / 'masks').mkdir()
    for name in ('sources.json', 'splits.json'):
        shutil.copyfile(corpus / 'config' / name, output / name)
    coverage = defaultdict(lambda: defaultdict(Counter))
    support = defaultdict(lambda: np.zeros(5, dtype=np.int64))
    repairs = []
    for index, row in enumerate(rows):
        source, terrain, split = (row[key] for key in ('source_id', 'terrain_id', 'split'))
        image_path, mask_path = (corpus / row[key] for key in ('image', 'mask'))
        with Image.open(mask_path) as mask:
            if mask.mode not in ('L', 'P'):
                raise ValueError(f'Expected indexed mask: {mask_path}')
            pixels = np.array(mask)
        if source == 'polar-sim-labels' and split != 'test' and np.any(pixels == 0):
            with Image.open(image_path) as image:
                clipped = np.asarray(image.convert('RGB')).min(axis=2) >= 250
            # Only remove regolith supervision where near-white clipping is
            # widespread. Preserve all existing positive class annotations.
            ambiguous = clipped & (pixels == 0)
            if ambiguous.sum() >= np.count_nonzero(pixels == 0) / 2:
                pixels[ambiguous] = 255
                revised = output / 'masks' / f'{index:05d}-{mask_path.name}'
                Image.fromarray(pixels).save(revised)
                repairs.append(dict(image=row['image'], mask=row['mask'], split=split,
                                    terrain=terrain, ignored_pixels=int(ambiguous.sum())))
                mask_path = revised
        counts = np.bincount(pixels.ravel(), minlength=256)
        if counts[5:255].any():
            raise ValueError(f'Unsupported label in {mask_path}')
        support[split] += counts[:5]
        group = f'{namespaces.get(source, source)}/{terrain}'
        for class_id, label in ((1, 'bedrock'), (3, 'crater')):
            if counts[class_id]:
                coverage[split][label][group] += 1
        row.update(image=Path(os.path.relpath(image_path, output)).as_posix(),
                   mask=Path(os.path.relpath(mask_path, output)).as_posix())
    with (output / 'manifest.csv').open('w', newline='', encoding='utf-8') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    report = dict(records=len(rows), repaired_masks=repairs, positive_frames_by_scene=coverage,
                  class_pixels={split: counts.tolist() for split, counts in support.items()},
                  scene_count_note='Source groups are manifest identifiers. Independence and domain must be confirmed before fold assignment.')
    (output / 'revision.json').write_text(json.dumps(report, indent=2) + '\n', encoding='utf-8')
    print(f'{len(rows)} records, {len(repairs)} masks repaired')
    for split, classes in coverage.items():
        for label, groups in classes.items():
            print(split, label, len(groups), 'manifest groups', sum(groups.values()), 'positive frames')


if __name__ == '__main__':
    main()
