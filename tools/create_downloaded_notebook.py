"""Generate the Kaggle notebook for the portable downloaded-data corpus."""
import json
from pathlib import Path


def main():
    root = Path(__file__).resolve().parents[1]
    cells = []

    def cell(kind, source):
        value = dict(cell_type=kind, id=f'cell-{len(cells):02d}', metadata={}, source=source.splitlines(keepends=True))
        if kind == 'code':
            value.update(execution_count=None, outputs=[])
        cells.append(value)

    cell('markdown', '''# P5-02 — new datasets, fresh five-class ResNet34 U-Net

Attach the private **kaggle-p502-new-datasets** dataset and enable a GPU.
The notebook accepts either the uploaded ZIP or its unpacked contents.
Official ImageNet encoder weights are bundled; Internet is not required. This is a fresh run:
the old notebook's checkpoints have different preprocessing/training settings.

New COCO/YOLO, LunarSeg and AI4Mars additions are training-only. The inherited
validation/test scenes remain fixed and sparse; their prior results have
already been inspected. These are engineering gates, not fresh independent
qualification evidence. Gates stay mIoU >=0.50 and each class >=0.25.
''')
    cell('code', '''import csv, hashlib, json, os, shutil, subprocess, sys, zipfile
from pathlib import Path
import torch, torchvision

# Edit these before saving a run.
RESUME = False
STOP_AFTER_EPOCH = 1
BATCH_SIZE = 8
IMAGE_SIZE = 384
WORKERS = 2
RESTORE_RUN_FROM = None  # set only when multiple prior run directories are attached
RUN_TEST = False
EXPORT_ZIP = False

assert torch.cuda.is_available(), "Enable a GPU in notebook settings."
print(torch.__version__, torchvision.__version__, torch.cuda.get_device_name(0))
INPUT = Path('/kaggle/input')
manifests = [p for p in INPUT.rglob('manifest.csv')
             if (p.parent / 'tools/train_downloaded_resnet.py').is_file()
             and (p.parent / 'config/gates-frozen.json').is_file()]
if manifests:
    assert len(manifests) == 1, f"Attach one training bundle: {manifests}"
    BUNDLE = manifests[0].parent
else:
    archives = list(INPUT.rglob('kaggle-p502-new-datasets.zip'))
    assert len(archives) == 1, f"Attach the training dataset ZIP: {archives}"
    BUNDLE = Path('/tmp/p502-dataset')
    marker = BUNDLE / '.extracted-from'
    archive = archives[0]
    fingerprint = f'{archive}:{archive.stat().st_size}:{archive.stat().st_mtime_ns}'
    if not marker.is_file() or marker.read_text() != fingerprint:
        if BUNDLE.exists():
            shutil.rmtree(BUNDLE)
        with zipfile.ZipFile(archive) as zipped:
            required = sum(item.file_size for item in zipped.infolist())
            assert shutil.disk_usage(BUNDLE.parent).free > required, 'Insufficient space to unpack dataset'
            print(f'Unpacking {archive.name} into {BUNDLE}', flush=True)
            zipped.extractall(BUNDLE)
        marker.write_text(fingerprint)
print('Bundle:', BUNDLE)
with (BUNDLE / 'manifest.csv').open() as stream:
    from collections import Counter
    counts = Counter(row['split'] for row in csv.DictReader(stream))
assert counts == {'train': 32104, 'validation': 746, 'test': 749}, counts
print('Records:', dict(counts))
print((BUNDLE / 'config/gates-frozen.json').read_text())
os.environ['TORCH_HOME'] = '/tmp/p502-torch-cache'
encoder = BUNDLE / 'weights/resnet34-b627a593.pth'
assert hashlib.sha256(encoder.read_bytes()).hexdigest().startswith('b627a593'), 'Encoder weight hash mismatch'
encoder_cache = Path(os.environ['TORCH_HOME']) / 'hub/checkpoints'
encoder_cache.mkdir(parents=True, exist_ok=True)
shutil.copy2(encoder, encoder_cache / encoder.name)
''')
    trainer_source = (root / 'tools/train_downloaded_resnet.py').read_text(encoding='utf-8')
    cell('code', '''# Install the trainer shipped with this notebook and the bundle's supporting modules.
TOOLS = Path('/tmp/p502-tools')
shutil.copytree(BUNDLE / 'tools', TOOLS, dirs_exist_ok=True)
sys.path.insert(0, str(TOOLS))
(TOOLS / 'train_downloaded_resnet.py').write_text(
''' + ''.join(f'    {line!r}\n' for line in trainer_source.splitlines(keepends=True)) + '''    , encoding='utf-8')
''')
    cell('markdown', '''## Train — no test predictions

The input is RGB, normalized using ImageNet mean/std. Images use bilinear
resizing; indexed masks use nearest-neighbor. Sparse validation masks retain
their stored resolution for scoring. No TTA, calibration, or copy-paste is used.
Checkpoint selection uses all five classes. A successful training process is
not a passing validation result: inspect `validation_gate_pass` below.

For an interrupted run, restore its whole output directory at the same RUN
path, set RESUME=True and preserve input paths, code and configuration.
The first chunk ends at epoch 1 of a fixed 40-epoch schedule. Use its measured
epoch time to choose a stop epoch that fits the next session. Keep the original
training dataset version and upload layout, attach the previous notebook output,
set RESUME=True and raise STOP_AFTER_EPOCH. Repeat until epoch 40.
''')
    cell('code', '''RUN = Path('/kaggle/working/p502-new-datasets-run-01')
assert 1 <= STOP_AFTER_EPOCH <= 40
if RESUME and not (RUN / 'last.pt').is_file():
    candidates = sorted({p.parent for p in INPUT.rglob('last.pt')
                         if (p.parent / 'best.pt').is_file()
                         and (p.parent / 'run-config.json').is_file()})
    preferred = [p for p in candidates if p.name == RUN.name]
    restore = Path(RESTORE_RUN_FROM) if RESTORE_RUN_FROM else (preferred[0] if len(preferred) == 1 else (candidates[0] if len(candidates) == 1 else None))
    assert restore is not None, f'Set RESTORE_RUN_FROM to one previous run directory: {candidates}'
    assert not RUN.exists(), f'Remove or move the incomplete run at {RUN} before restoring'
    shutil.copytree(restore, RUN)
CORPUS = Path('/tmp/p502-corpus')
command = [sys.executable, '-u', str(TOOLS / 'train_downloaded_resnet.py'),
           '--corpus', str(CORPUS),
           '--gates', str(BUNDLE / 'config/gates-frozen.json'),
           '--output', str(RUN), '--epochs', '40', '--batch-size', str(BATCH_SIZE),
           '--size', str(IMAGE_SIZE), '--workers', str(WORKERS),
           '--stop-after-epoch', str(STOP_AFTER_EPOCH)]
# The portable package puts policy JSONs in config/; create a lightweight
# canonical corpus directory with stable absolute paths, without copying images.
CORPUS.mkdir(exist_ok=True)
portable = list(csv.DictReader((BUNDLE / 'manifest.csv').open()))
for row in portable:
    for field in ('image', 'mask'):
        row[field] = str(BUNDLE / row[field])
with (CORPUS / 'manifest.csv').open('w', newline='') as stream:
    writer = csv.DictWriter(stream, fieldnames=['image','mask','source_id','terrain_id','split'])
    writer.writeheader(); writer.writerows(portable)
for name in ('sources.json', 'splits.json'):
    shutil.copy2(BUNDLE / 'config' / name, CORPUS / name)
if RESUME:
    command.append('--resume')
print(' '.join(command))
subprocess.run(command, check=True)
''')
    cell('code', '''validation = json.loads((RUN / 'validation-metrics.json').read_text())
history = json.loads((RUN / 'history.json').read_text())
from statistics import median
seconds_per_epoch = median(row['seconds'] for row in history[-3:])
print('Completed epoch:', history[-1]['epoch'], '/ 40')
print(f'Recent epoch time: {seconds_per_epoch / 60:.1f} minutes')
print(f'Estimated remaining training time: {(40 - history[-1]["epoch"]) * seconds_per_epoch / 3600:.1f} hours')
print('Selected epoch:', validation['epoch'])
print('Validation gate passed:', validation['validation_gate_pass'])
print(json.dumps(validation['metrics'], indent=2))
''')
    cell('markdown', '''## Visual check — same inference as the validation metric

Gray=regolith, red=bedrock, blue=rock, orange=crater, black=shadow.
Blank truth pixels are ignored, not background ground truth. Predictions over
ignored pixels are shown but do not contribute to IoU.
''')
    cell('code', '''import numpy as np
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap
from PIL import Image
from train_downloaded_resnet import UNetR34, Validation
from train_perception import load_records
from torch.nn import functional as F

records = load_records(CORPUS / 'manifest.csv', CORPUS,
                       CORPUS / 'splits.json', CORPUS / 'sources.json')
ck = torch.load(RUN / 'best.pt', map_location='cpu', weights_only=True)
net = UNetR34().cuda().eval()
net.load_state_dict(ck['state_dict'])
view = Validation(records['validation'], ck['image_size'])
rare = [i for i, r in enumerate(records['validation'])
        if np.isin(np.asarray(Image.open(r['mask'])), [1,3]).any()]
indices = list(dict.fromkeys(rare + [0, len(view)//2, len(view)-1]))[:6]
cmap = ListedColormap(['#999999','#e41a1c','#377eb8','#ff7f00','#000000'])
fig, axes = plt.subplots(len(indices), 3, figsize=(12, 3*len(indices)), squeeze=False)
for ax, i in zip(axes, indices):
    x, target = view[i]
    with torch.inference_mode(), torch.autocast('cuda'):
        logits = net(x[None].cuda())
    pred = F.interpolate(logits.float(), size=target.shape, mode='bilinear', align_corners=False).argmax(1)[0].cpu().numpy()
    truth = target.numpy().astype(float); truth[truth == 255] = np.nan
    ax[0].imshow(Image.open(records['validation'][i]['image'])); ax[0].set_title(f'Validation {i}')
    ax[1].imshow(truth, cmap=cmap, vmin=0, vmax=4); ax[1].set_title('Truth — blank=ignore')
    ax[2].imshow(pred, cmap=cmap, vmin=0, vmax=4); ax[2].set_title('Prediction')
    for item in ax: item.axis('off')
plt.tight_layout(); plt.show()
del net
torch.cuda.empty_cache()
''')
    cell('markdown', '''## Final test — leave disabled during development

Choose the final model using validation, freeze the pipeline, then run once.
The evaluator requires passing validation, unchanged input/code/gate hashes,
and refuses to overwrite an existing test result. Previously inspected test
scenes remain historical engineering evidence; they are not a new blind test.
''')
    cell('code', '''if RUN_TEST and (history[-1]['epoch'] != 40 or not validation['validation_gate_pass']):
    print('Test deferred until epoch 40 and a passing validation result.')
elif RUN_TEST and (RUN / 'test-metrics.json').is_file():
    print('Existing test result:', RUN / 'test-metrics.json')
elif RUN_TEST:
    test = subprocess.run([sys.executable, '-u', str(TOOLS / 'evaluate_downloaded_resnet.py'),
                           '--corpus', str(CORPUS), '--gates', str(BUNDLE / 'config/gates-frozen.json'),
                           '--run', str(RUN)])
    if test.returncode and not (RUN / 'test-metrics.json').is_file():
        test.check_returncode()
''')
    cell('code', '''with (RUN / 'environment.txt').open('w') as stream:
    subprocess.run([sys.executable, '-m', 'pip', 'freeze'], stdout=stream, check=True)
if EXPORT_ZIP:
    output = Path('/kaggle/working/p502-new-datasets-delivery.zip')
    with zipfile.ZipFile(output, 'w', allowZip64=True) as delivery:
        for folder, prefix in [(RUN, 'run'), (TOOLS, 'tools'), (BUNDLE / 'config', 'config')]:
            for path in sorted(folder.iterdir()):
                if path.is_file():
                    delivery.write(path, f'{prefix}/{path.name}')
        delivery.write(BUNDLE / 'manifest.csv', 'manifest.csv')
    print(output)
''')
    notebook = dict(cells=cells, metadata=dict(kernelspec=dict(display_name='Python 3', language='python', name='python3'),
                                              language_info=dict(name='python')), nbformat=4, nbformat_minor=5)
    output = root / 'notebooks/p502-new-datasets-kaggle.ipynb'
    output.parent.mkdir(exist_ok=True)
    output.write_text(json.dumps(notebook, indent=2) + '\n')
    print(output)


if __name__ == '__main__':
    main()
