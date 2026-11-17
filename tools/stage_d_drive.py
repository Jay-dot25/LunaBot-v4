"""Resume-safe staging of the user's downloaded datasets on D: (run in WSL)."""
import hashlib
import json
import shutil
import subprocess
import zipfile
from pathlib import Path


def digest(path):
    h = hashlib.sha256()
    with path.open('rb') as f:
        for b in iter(lambda: f.read(8 * 1024 * 1024), b''):
            h.update(b)
    return h.hexdigest()


def main():
    root = Path('/mnt/d/data/lunabot-datasets')
    raw = root / 'archives'
    raw.mkdir(parents=True, exist_ok=True)
    report = {}
    inputs = {
        'coco': Path('/mnt/c/Users/HP/Downloads/Dataset_coco.zip'),
        'yolo': Path('/mnt/c/Users/HP/Downloads/Dataset_yolo.zip'),
        'lunarseg': Path('/mnt/c/Users/HP/Downloads/LunarSeg.rar'),
        'ai4mars': Path('/mnt/d/data/ai4mars-dataset-merged-0.6.zip'),
    }
    for name, original in inputs.items():
        archive = raw / original.name if name != 'ai4mars' else original
        if original != archive:
            if not archive.exists():
                shutil.copy2(original, archive)
            sha = digest(archive)
            if original.exists():
                if sha != digest(original):
                    raise ValueError(f'Copy verification failed: {original}')
                # Explicitly named downloads only, after byte verification.
                original.unlink()
        else:
            sha = digest(archive)
        dest = root / name
        dest.mkdir(exist_ok=True)
        print(f'STAGING {name}: {archive}', flush=True)
        if name == 'ai4mars':
            existing = root / '20261110-012024-584472/ai4mars-001'
            if existing.is_dir():
                dest = existing
        if archive.suffix == '.zip':
            with zipfile.ZipFile(archive) as z:
                count = 0
                for member in z.infolist():
                    path = (dest / member.filename).resolve()
                    if not path.is_relative_to(dest.resolve()):
                        raise ValueError('Unsafe archive path')
                    if member.is_dir():
                        continue
                    if path.is_file() and path.stat().st_size == member.file_size:
                        continue
                    z.extract(member, dest)
                    count += 1
                    if count % 5000 == 0:
                        print(name, 'extracted', count, flush=True)
                total = sum(not m.is_dir() for m in z.infolist())
        else:
            subprocess.run(['unrar', 'x', '-o-', '-idq', str(archive), str(dest) + '/'], check=True)
            total = None
        report[name] = {'archive': str(archive), 'archive_sha256': sha,
                        'extracted_root': str(dest), 'archive_files': total}
        (root / 'staging.json').write_text(json.dumps(report, indent=2) + '\n')
        print(f'READY {name}: {dest}', flush=True)


if __name__ == '__main__':
    main()
