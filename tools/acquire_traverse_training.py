"""Acquire a fixed training-only NASA Traverse sample with member CRC and SHA."""
from concurrent.futures import ThreadPoolExecutor
import hashlib
import json
from pathlib import Path
import struct
import zlib

from acquire_baseprod import RemoteArchive

ROOT = Path(__file__).resolve().parents[1]


def main():
    out = ROOT/'.build/p502-traverse'
    inventory = json.loads((out/'inventory.json').read_text())
    selected = [r for r in inventory['members'] if r['name'].startswith('View1/Traverse1/')
                and any(r['name'].endswith(f'camL_{e:03d}ms.png') for e in [10,25,50])
                and int(r['name'].split('/')[2][:-1]) % 2 == 0]
    # Restrict by filename first: directory records have no position component.
    def fetch(info):
        name = info['name']
        dest = out/(hashlib.sha256(name.encode()).hexdigest()+'.png')
        if dest.exists():
            content = dest.read_bytes()
        else:
            remote = RemoteArchive(inventory['url'],inventory['size'])
            remote.seek(info['offset'])
            header = remote.read(30)
            if header[:4] != b'PK\x03\x04':
                raise ValueError('bad member header')
            name_size, extra_size = struct.unpack_from('<HH',header,26)
            remote.seek(info['offset']+30+name_size+extra_size)
            content=zlib.decompress(remote.read(info['compressed']),-15)
        if len(content)!=info['size'] or zlib.crc32(content)!=info['crc']:
            raise ValueError('member CRC/size mismatch')
        dest.write_bytes(content)
        print(name,flush=True)
        return dict(member=name,path=str(dest),sha256=hashlib.sha256(content).hexdigest(),crc32=info['crc'])
    with ThreadPoolExecutor(max_workers=6) as pool:
        acquired=list(pool.map(fetch,selected))
    (out/'acquired.json').write_text(json.dumps(dict(url=inventory['url'],scope='entire acquisition training-only',members=acquired),indent=2)+'\n')


if __name__=='__main__':
    main()
