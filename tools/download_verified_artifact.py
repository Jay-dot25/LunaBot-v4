"""Resume a large HTTPS artifact in bounded ranges and verify its SHA-256."""
import argparse
import concurrent.futures
import hashlib
import os
from pathlib import Path
import time
import urllib.request


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--url', required=True)
    p.add_argument('--sha256', required=True)
    p.add_argument('--output', type=Path, required=True)
    a = p.parse_args()
    if not a.url.startswith('https://'):
        p.error('HTTPS required')
    req = urllib.request.Request(a.url, headers={'Range': 'bytes=0-0'})
    with urllib.request.urlopen(req, timeout=45) as response:
        total = int(response.headers['Content-Range'].split('/')[-1])
    parts = a.output.parent / (a.output.name + '.parts')
    parts.mkdir(parents=True, exist_ok=True)
    size = 4 * 1024 * 1024
    def fetch(start):
        end = min(start + size, total) - 1
        dest = parts / str(start)
        if dest.exists() and dest.stat().st_size == end-start+1:
            return
        temp = dest.with_suffix('.tmp')
        if temp.exists() and temp.stat().st_size > end-start+1:
            raise ValueError('oversized partial range')
        failures = 0
        while True:
            try:
                current = start + (temp.stat().st_size if temp.exists() else 0)
                if current == end+1:
                    temp.replace(dest)
                    return
                request_end = min(current + 1024*1024 - 1, end)
                request = urllib.request.Request(a.url, headers={'Range': f'bytes={current}-{request_end}'})
                with urllib.request.urlopen(request, timeout=45) as r, temp.open('ab', buffering=0) as stream:
                    if r.status != 206 or r.headers.get('Content-Range') != f'bytes {current}-{request_end}/{total}':
                        raise ValueError('server returned a different range')
                    remaining = request_end-current+1
                    while remaining:
                        block = r.read1(min(65536, remaining))
                        if not block:
                            raise ValueError('truncated range; retaining received bytes')
                        stream.write(block)
                        remaining -= len(block)
                failures = 0
            except Exception as exc:
                failures += 1
                print(f'range {start}: retry {failures}, retained {temp.stat().st_size if temp.exists() else 0} bytes ({type(exc).__name__})', flush=True)
                if failures >= 20:
                    raise
                time.sleep(min(2**min(failures, 3), 8))
    starts = list(range(0, total, size))
    with concurrent.futures.ThreadPoolExecutor(max_workers=8) as executor:
        for n, _ in enumerate(executor.map(fetch, starts), 1):
            if n % 10 == 0 or n == len(starts):
                print(f'{n}/{len(starts)} ranges verified', flush=True)
    tmp = a.output.with_suffix(a.output.suffix + '.tmp')
    digest = hashlib.sha256()
    with tmp.open('wb') as out:
        for start in starts:
            data = (parts / str(start)).read_bytes()
            digest.update(data)
            out.write(data)
        out.flush()
        os.fsync(out.fileno())
    if digest.hexdigest() != a.sha256:
        raise ValueError(f'SHA-256 mismatch: {digest.hexdigest()}')
    tmp.replace(a.output)
    print(f'PASS SHA-256 {a.sha256}: {a.output}', flush=True)


if __name__ == '__main__':
    main()
