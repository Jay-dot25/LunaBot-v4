"""Check interrupted-range recovery without using the network."""
import hashlib
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'tools'))
import download_verified_artifact as downloader


class DownloadTests(unittest.TestCase):
    def test_retains_received_bytes_after_socket_failure(self):
        payload=bytes(range(256))*9000
        calls=[]
        failed=False
        class Response:
            status=206
            def __init__(self,start,end,interrupt=False):
                self.position,self.end=start,end
                self.interrupt=interrupt
                self.reads=0
                self.headers={'Content-Range':f'bytes {start}-{end}/{len(payload)}'}
            def __enter__(self):return self
            def __exit__(self,*args):pass
            def read1(self,n):
                self.reads+=1
                if self.interrupt and self.reads==2:raise TimeoutError('simulated mid-body timeout')
                n=min(n,self.end-self.position+1)
                block=payload[self.position:self.position+n]
                self.position+=n
                return block
        def open_request(request,timeout):
            nonlocal failed
            start,end=map(int,request.get_header('Range').split('=')[1].split('-'))
            calls.append((start,end))
            interrupt=end>0 and not failed
            if interrupt:failed=True
            return Response(start,end,interrupt)
        with tempfile.TemporaryDirectory() as directory:
            dest=Path(directory)/'artifact.bin'
            args=['download','--url','https://example.test/artifact','--sha256',hashlib.sha256(payload).hexdigest(),'--output',str(dest)]
            with patch.object(sys,'argv',args),patch.object(downloader.urllib.request,'urlopen',side_effect=open_request),patch.object(downloader.time,'sleep'):
                downloader.main()
            self.assertEqual(payload,dest.read_bytes())
            self.assertTrue(any(start==65536 for start,_ in calls),calls)


if __name__=='__main__':unittest.main()
