"""Fetch the public BASEPROD ZIP directory and selected members using ranges.

Pin the Zenodo record and retain metadata and member SHA-256. Downloads only
requested members; no extraction paths are taken from the remote archive.
"""
import argparse
import hashlib
import io
import json
import struct
import zlib
from concurrent.futures import ThreadPoolExecutor, as_completed
from pathlib import Path
import urllib.request
from zipfile import ZipFile

RECORD = 'https://zenodo.org/api/records/20289198'


class RemoteArchive(io.RawIOBase):
    def __init__(self, url, size):
        self.url, self.size, self.position = url, size, 0
        self.cache_start, self.cache = 0, b''

    def seekable(self):
        return True

    def readable(self):
        return True

    def tell(self):
        return self.position

    def seek(self, offset, whence=0):
        self.position = offset + (self.position if whence == 1 else self.size if whence == 2 else 0)
        return self.position

    def read(self, size=-1):
        size = min(self.size - self.position, size if size >= 0 else self.size)
        if size <= 0:
            return b''
        if size > 512 * 1024:
            chunks = []
            remaining = size
            while remaining:
                chunk = self.read(min(remaining, 512 * 1024))
                chunks.append(chunk)
                remaining -= len(chunk)
            return b''.join(chunks)
        if self.cache_start <= self.position and self.position + size <= self.cache_start + len(self.cache):
            offset = self.position - self.cache_start
            self.position += size
            return self.cache[offset:offset + size]
        if size > 32 * 1024 * 1024:
            raise ValueError('refusing an unexpectedly large range')
        start = self.position
        end = min(self.size - 1, start + max(size, 512 * 1024) - 1)
        content = b''
        for attempt in range(3):
            current = start + len(content)
            request = urllib.request.Request(self.url + f'?range_start={current}&range_end={end}&attempt={attempt}',
                                             headers={'Range': f'bytes={current}-{end}'})
            with urllib.request.urlopen(request, timeout=60) as response:
                if response.status != 206 or not response.headers.get('Content-Range', '').startswith(f'bytes {current}-{end}/'):
                    raise ValueError('server did not honor byte range')
                content += response.read(end - current + 2)
            if len(content) >= end - start + 1:
                break
        if len(content) != end - start + 1:
            raise ValueError(f'truncated or oversized range response: {start}-{end}, '
                             f'expected {end-start+1}, got {len(content)}')
        self.position += size
        self.cache_start, self.cache = start, content
        return content[:size]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--members', type=Path, help='JSON array of exact archive member names')
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    with urllib.request.urlopen(RECORD, timeout=60) as response:
        metadata_bytes = response.read()
    metadata = json.loads(metadata_bytes)
    if metadata['metadata']['license']['id'] != 'cc-by-4.0':
        raise ValueError('source license differs from the approved CC-BY-4.0 record')
    (args.output / 'zenodo-record.json').write_bytes(metadata_bytes)
    entry = metadata['files'][0]
    remote = RemoteArchive(entry['links']['self'], entry['size'])
    with ZipFile(remote) as archive:
        inventory = [{'name': i.filename, 'size': i.file_size, 'compressed': i.compress_size,
                      'crc': i.CRC} for i in archive.infolist()]
        (args.output / 'archive-inventory.json').write_text(json.dumps(inventory, indent=2) + '\n')
        print(json.dumps({'record': metadata['id'], 'members': len(inventory)}), flush=True)
        if args.members:
            requested = json.loads(args.members.read_text())
            provenance = []
            def fetch_member(name):
                # Store by hash to avoid zip-slip and ambiguous duplicate names.
                destination = args.output / 'members' / (hashlib.sha256(name.encode()).hexdigest() + Path(name).suffix)
                destination.parent.mkdir(exist_ok=True)
                if destination.exists():
                    content = destination.read_bytes()
                else:
                    info = archive.getinfo(name)
                    member_remote = RemoteArchive(entry['links']['self'], entry['size'])
                    member_remote.seek(info.header_offset)
                    block = member_remote.read(info.compress_size + 30 + len(name.encode()) + 65535)
                    if block[:4] != b'PK\x03\x04':
                        raise ValueError('bad ZIP local header')
                    name_size, extra_size = struct.unpack_from('<HH', block, 26)
                    start = 30 + name_size + extra_size
                    compressed = block[start:start + info.compress_size]
                    if info.compress_type == 8:
                        content = zlib.decompress(compressed, -15)
                    elif info.compress_type == 0:
                        content = compressed
                    else:
                        raise ValueError('unsupported compression method')
                info = archive.getinfo(name)
                if len(content) != info.file_size or zlib.crc32(content) != info.CRC:
                    raise ValueError(f'member size/CRC mismatch: {name}')
                if not destination.exists():
                    destination.write_bytes(content)
                return {'member': name, 'path': str(destination.resolve()),
                        'sha256': hashlib.sha256(content).hexdigest(), 'bytes': len(content)}
            with ThreadPoolExecutor(max_workers=6) as executor:
                for future in as_completed([executor.submit(fetch_member, name) for name in requested]):
                    member = future.result()
                    provenance.append(member)
                    (args.output / 'downloaded-members.json').write_text(json.dumps(provenance, indent=2) + '\n')
                    print(f'{len(provenance)}/{len(requested)} {member["member"]}', flush=True)


if __name__ == '__main__':
    main()
