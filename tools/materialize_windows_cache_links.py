"""Replace Linux-only symlinks in the D: reviewed-cache snapshot with verified bytes."""
import csv
import hashlib
import os
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path


def main():
    cache = Path('/mnt/d/lunabot-v4/.build/p502-cache-reviewed-v2')
    with Path('/mnt/d/data/lunabot-datasets/canonical-20261110/manifest.csv').open() as stream:
        rows = list(csv.DictReader(stream))
    paths = sorted({Path(row[field]) for row in rows for field in ('image', 'mask')
                    if Path(row[field]).parent == cache})

    def convert(path):
        if not path.is_symlink():
            return 0
        data = path.read_bytes()
        temp = path.with_name(path.name + '.windows-copy.tmp')
        if temp.exists():
            raise ValueError(f'Unexpected temporary file: {temp}')
        temp.write_bytes(data)
        if hashlib.sha256(temp.read_bytes()).digest() != hashlib.sha256(data).digest():
            raise ValueError(f'Copy hash mismatch: {path}')
        # Replace the link itself, not its Linux target.
        os.replace(temp, path)
        return 1

    count = 0
    with ThreadPoolExecutor(max_workers=8) as pool:
        for i, value in enumerate(pool.map(convert, paths), 1):
            count += value
            if i % 2000 == 0:
                print('Checked', i, '/', len(paths), 'materialized', count, flush=True)
    print('Verified Windows-readable cache files:', count, flush=True)


if __name__ == '__main__':
    main()
