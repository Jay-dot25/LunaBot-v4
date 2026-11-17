#!/usr/bin/env python3
"""Bounded, cleanup-safe orchestration of real P2-03 fault injection."""
from pathlib import Path
import argparse,os,subprocess,time,signal,json
def main():
    ap=argparse.ArgumentParser(); ap.add_argument('--evidence',type=Path,required=True); a=ap.parse_args()
    root=Path(__file__).resolve().parents[1]
    d=a.evidence.resolve(); d.mkdir(parents=True,exist_ok=False)
    code=1; launcher_exit=None
    with (d/'console.log').open('w') as log:
        p=subprocess.Popen(['bash',str(root/'scripts/launch-b.sh')],cwd=root,
             env=dict(os.environ,HEADLESS='1',HOLD='1',EVIDENCE_DIR=str(d)),stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        try:
            deadline=time.monotonic()+150
            while time.monotonic()<deadline:
                if p.poll() is not None: raise RuntimeError('launcher exited before ready')
                if 'HEADLESS HOLD READY' in (d/'console.log').read_text(): break
                time.sleep(.25)
            else: raise RuntimeError('startup timed out')
            proc=subprocess.run(['python3',str(root/'tests/test_safety_faults_live.py'),
                  '--stack-evidence',str(d),'--output',str(d/'faults.json')],
                  capture_output=True,text=True,timeout=120)
            (d/'test.log').write_text(proc.stdout+proc.stderr)
            code=proc.returncode
        finally:
            if p.poll() is None: os.killpg(p.pid,signal.SIGINT)
            try: launcher_exit=p.wait(timeout=40)
            except subprocess.TimeoutExpired:
                os.killpg(p.pid,signal.SIGKILL); launcher_exit=p.wait()
            (d/'exit.json').write_text(json.dumps({'test_exit':code,'launcher_exit':launcher_exit})+'\n')
    print('test exit',code,'launcher exit',launcher_exit,flush=True)
    raise SystemExit(code)
if __name__=='__main__': main()
