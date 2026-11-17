"""Independently verify frozen cached pairs, lineage and cross-split image hashes."""
import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path

from PIL import Image

from train_perception import ROOT, audit_class_support, load_records, sha256_file


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--cache', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    a = p.parse_args()
    records = load_records(a.cache/'manifest.csv', a.cache,
                           ROOT/'config/perception/splits.json', ROOT/'config/perception/sources.json')
    report = json.loads((a.cache/'cache-report.json').read_text())
    lineage = {hashlib.sha256(r['source_image'].encode()).hexdigest(): r for r in report['lineage']}
    seen, collisions, errors = {}, [], []
    source_splits = {}
    for split, rows in records.items():
        source_splits[split] = dict(Counter(r['source_id'] for r in rows))
        for row in rows:
            image, mask = Path(row['image']), Path(row['mask'])
            digest = sha256_file(image)
            if digest in seen and seen[digest] != split:
                collisions.append({'image':str(image), 'splits':[seen[digest], split]})
            seen[digest] = split
            origin = lineage.get(image.stem)
            if origin is None or digest != origin['image_sha256'] or sha256_file(mask) != origin['mask_sha256']:
                errors.append(f'cached lineage mismatch: {image}')
            elif any(sha256_file(Path(origin[f'source_{field}'])) != origin[f'source_{field}_sha256'] for field in ('image','mask')):
                errors.append(f'original lineage mismatch: {image}')
            with Image.open(image) as im, Image.open(mask) as ma:
                if im.size != ma.size or im.size != (report['image_size'],)*2:
                    errors.append(f'pair dimensions mismatch: {image}')
    support = audit_class_support(records)
    if support != report['resized_class_pixel_support']:
        errors.append('cached class histogram changed')
    result = dict(result='PASS' if not errors and not collisions else 'FAIL',
                  manifest_sha256=sha256_file(a.cache/'manifest.csv'),
                  original_manifest_sha256=report['input_manifest_sha256'],
                  sources_sha256=sha256_file(ROOT/'config/perception/sources.json'),
                  splits_sha256=sha256_file(ROOT/'config/perception/splits.json'),
                  records={k:len(v) for k,v in records.items()}, source_records=source_splits,
                  pixel_support=support, original_and_cached_lineage_verified=not errors,
                  cross_split_identical_images=collisions, errors=errors)
    a.output.parent.mkdir(parents=True,exist_ok=True)
    a.output.write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2))
    return 0 if result['result']=='PASS' else 1


if __name__ == '__main__':
    raise SystemExit(main())
