"""Build conservative, source-separated bedrock evaluation additions.

Apollo 15 Hadley Rille stays wholly in validation. The entire sampled Bardenas
acquisition stays in test. No added evaluation frame is used for optimization.
These additions support a bounded prototype assessment, not a deployment claim.
"""
import argparse
import csv
import hashlib
import json
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def remap_baseprod(mask):
    if set(np.unique(mask)) - set(range(8)):
        raise ValueError('unsupported BASEPROD mask ID')
    out = np.full(mask.shape, 255, dtype=np.uint8)
    for original, canonical in ((3, 1), (4, 0), (6, 2)):
        out[mask == original] = canonical
    # Compact soil, pebbles, plants and void are not guessed into our ontology.
    return out


APOLLO = {
    'AS15-89-12115HR.jpg': [
        (1, [[.34,.528],[.375,.525],[.382,.544],[.345,.55]]),
        (0, [[.40,.27],[.47,.27],[.47,.30],[.40,.30]])],
    'AS15-89-12157HR.jpg': [
        (1, [[.43,.30],[.47,.307],[.48,.326],[.438,.324]]),
        (0, [[.40,.15],[.47,.15],[.47,.18],[.40,.18]])],
}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--baseprod', type=Path, required=True)
    parser.add_argument('--apollo', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    metadata = json.loads((args.baseprod / 'zenodo-record.json').read_text())
    if metadata['id'] != 20289198 or metadata['metadata']['license']['id'] != 'cc-by-4.0':
        raise ValueError('BASEPROD record/license changed')
    members = {r['member']: r for r in json.loads((args.baseprod / 'downloaded-members.json').read_text())}
    rows, provenance = [], []
    histogram = {'validation': [0]*5, 'test': [0]*5}
    for name, record in sorted(members.items()):
        if '/ORIGINAL/COLOR_IMGS/' not in name:
            continue
        mask_name = name.replace('/COLOR_IMGS/', '/MASK_RAWS/').replace('.png', '_mask.png')
        mask_record = members[mask_name]
        image_path, mask_path = Path(record['path']), Path(mask_record['path'])
        if sha(image_path) != record['sha256'] or sha(mask_path) != mask_record['sha256']:
            raise ValueError('acquired member hash mismatch')
        with Image.open(image_path) as image, Image.open(mask_path) as mask:
            if mask.mode not in ('L', 'P') or image.size != mask.size:
                raise ValueError('BASEPROD mask encoding or dimensions differ')
            target = remap_baseprod(np.asarray(mask))
        destination = args.output / (Path(name).stem + '-mask.png')
        Image.fromarray(target).save(destination)
        rows.append({'image': str(image_path.resolve()), 'mask': str(destination.resolve()),
                     'source_id': 'baseprod-bedrock-evaluation', 'terrain_id': 'bardenas-2023', 'split': 'test'})
        provenance.append({'image': str(image_path), 'image_sha256': sha(image_path),
                           'mask_member': mask_name, 'source_mask_sha256': sha(mask_path),
                           'canonical_mask_sha256': sha(destination)})
        for i in range(5):
            histogram['test'][i] += int((target == i).sum())
    for filename, polygons in APOLLO.items():
        image_path = args.apollo / filename
        with Image.open(image_path) as image:
            size = image.size
            overlay = image.convert('RGB')
        mask = Image.new('L', size, 255)
        draw = ImageDraw.Draw(mask)
        overlay_draw = ImageDraw.Draw(overlay)
        for class_id, polygon in polygons:
            points = [(round(x*(size[0]-1)), round(y*(size[1]-1))) for x,y in polygon]
            draw.polygon(points, fill=class_id)
            overlay_draw.polygon(points, outline='magenta' if class_id == 1 else 'cyan', width=5)
        destination = args.output / (image_path.stem + '-mask.png')
        mask.save(destination)
        overlay.save(args.output / (image_path.stem + '-review.png'))
        rows.append({'image': str(image_path.resolve()), 'mask': str(destination.resolve()),
                     'source_id': 'apollo15-bedrock-evaluation', 'terrain_id': 'hadley-rille-9a', 'split': 'validation'})
        provenance.append({'image': str(image_path), 'image_sha256': sha(image_path),
                           'canonical_mask_sha256': sha(destination), 'polygons_normalized': polygons,
                           'reviewer': 'codex-on-behalf-of-jay',
                           'scope': 'Conservative illuminated bedrock and regolith patches only; all other pixels ignored.'})
        target = np.asarray(mask)
        # Keep only clearly illuminated portions of the geological outcrop.
        # Dark fissures are uncertain/ignored, never inferred to be shadow.
        with Image.open(image_path) as photograph:
            brightness = np.asarray(photograph.convert('L'))
        target = target.copy()
        target[(target == 1) & (brightness < 150)] = 255
        Image.fromarray(target).save(destination)
        provenance[-1]['canonical_mask_sha256'] = sha(destination)
        provenance[-1]['uncertain_dark_outcrop_pixels'] = 'ignored below luminance 150; no shadow class inferred'
        for i in range(5):
            histogram['validation'][i] += int((target == i).sum())
    with (args.output / 'manifest.csv').open('w', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    report = {'schema_version': 1, 'records': len(rows), 'pixel_support': histogram,
              'source_provenance': provenance, 'all_additions_evaluation_only': True,
              'limitations': ['Only two Apollo telephoto frames from one scene; sparse manual patches.',
                              '48 BASEPROD pairs sampled uniformly every twentieth file before model evaluation.',
                              'Bardenas is a Mars analogue; Apollo is real lunar imagery, with different cameras/scales.',
                              'This is a bounded prototype evaluation, not general lunar bedrock qualification.'],
              'mapping': {'3-bedrock': 1, '4-sandy': 0, '6-rock': 2, 'other': 255}}
    (args.output / 'report.json').write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps({k: report[k] for k in ('records','pixel_support','limitations')}, indent=2))


if __name__ == '__main__':
    main()
