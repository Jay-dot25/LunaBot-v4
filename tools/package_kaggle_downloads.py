"""Package the frozen D: canonical corpus directly into a portable Kaggle ZIP."""
import argparse
import csv
import hashlib
import io
import json
import zipfile
from pathlib import Path


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--corpus', type=Path, required=True)
    p.add_argument('--repo', type=Path, default=Path('/mnt/d/lunabot-v4'))
    p.add_argument('--output', type=Path, required=True)
    a = p.parse_args()
    audit = json.loads((a.corpus / 'audit.json').read_text())
    if audit['result'] != 'PASS' or any(hashlib.sha256((a.corpus / name).read_bytes()).hexdigest() != digest
                                      for name, digest in audit['configuration_sha256'].items()):
        raise ValueError('Corpus audit failed or configuration changed')
    a.output.parent.mkdir(parents=True, exist_ok=True)
    with (a.corpus / 'manifest.csv').open() as stream:
        rows = list(csv.DictReader(stream))
    fields = ['image', 'mask', 'source_id', 'terrain_id', 'split']
    files = {}
    for row in rows:
        for field in ['image', 'mask']:
            path = Path(row[field])
            key = hashlib.sha256(str(path).encode()).hexdigest()
            relative = 'data/' + key + path.suffix
            files[relative] = path
            row[field] = relative
    manifest = io.StringIO(newline='')
    writer = csv.DictWriter(manifest, fieldnames=fields)
    writer.writeheader(); writer.writerows(rows)
    temp = a.output.with_suffix('.partial.zip')
    with zipfile.ZipFile(temp, 'w', compression=zipfile.ZIP_STORED, allowZip64=True) as z:
        z.writestr('manifest.csv', manifest.getvalue())
        for name in ['sources.json', 'splits.json', 'materialization.json', 'audit.json']:
            z.write(a.corpus / name, 'config/' + name)
        frozen = a.repo / 'evidence/runs/20261107-p502/P5-02'
        z.write(frozen / 'gates-frozen.json', 'config/gates-frozen.json')
        z.write(a.repo / 'config/perception/classes.json', 'config/classes.json')
        for name in ['train_downloaded_resnet.py', 'evaluate_downloaded_resnet.py', 'train_perception.py', 'perception_metrics.py']:
            z.write(a.repo / 'tools' / name, 'tools/' + name)
        z.write(a.repo / 'docs/d-drive-training.md', 'README-project.md')
        z.write('/mnt/d/data/lunabot-torch-cache/hub/checkpoints/resnet34-b627a593.pth', 'weights/resnet34-b627a593.pth')
        provenance = a.corpus.parent / 'staging.json'
        z.write(provenance, 'config/archive-provenance.json')
        for i, (relative, path) in enumerate(sorted(files.items()), 1):
            z.write(path, relative)
            if i % 2000 == 0:
                print(f'Packed {i}/{len(files)} files', flush=True)
    temp.replace(a.output)
    digest = hashlib.sha256()
    with a.output.open('rb') as stream:
        for block in iter(lambda: stream.read(8 * 1024 * 1024), b''):
            digest.update(block)
    summary = dict(archive=str(a.output), sha256=digest.hexdigest(), records=len(rows), files=len(files), bytes=a.output.stat().st_size)
    a.output.with_suffix('.json').write_text(json.dumps(summary, indent=2) + '\n')
    print(json.dumps(summary), flush=True)


if __name__ == '__main__':
    main()
