"""Fresh RGB ResNet34 U-Net development run on the D: canonical manifest.

Validation selects checkpoints using all five classes; test inference is absent.
Frozen acceptance gates are recorded, never changed. Run with the D: venv.
"""
import argparse
import copy
import hashlib
import json
import os
import random
import math
import time
from collections import Counter
from pathlib import Path

import numpy as np
from PIL import Image
import torch
from torch import nn
from torch.nn import functional as F
from torch.utils.data import DataLoader, Dataset, WeightedRandomSampler
import torchvision

from train_perception import load_records, load_score_gates, audit_class_support, sha256_file, CLASS_NAMES
from perception_metrics import summarize_confusion


class Block(nn.Module):
    def __init__(self, incoming, outgoing):
        super().__init__()
        self.f = nn.Sequential(nn.Conv2d(incoming, outgoing, 3, padding=1, bias=False), nn.BatchNorm2d(outgoing), nn.ReLU(True),
                               nn.Conv2d(outgoing, outgoing, 3, padding=1, bias=False), nn.BatchNorm2d(outgoing), nn.ReLU(True))

    def forward(self, x):
        return self.f(x)


class UNetR34(nn.Module):
    def __init__(self, pretrained=False):
        super().__init__()
        r = torchvision.models.resnet34(weights=torchvision.models.ResNet34_Weights.IMAGENET1K_V1 if pretrained else None)
        self.stem = nn.Sequential(r.conv1, r.bn1, r.relu)
        self.pool = r.maxpool
        self.l1, self.l2, self.l3, self.l4 = r.layer1, r.layer2, r.layer3, r.layer4
        self.d4, self.d3 = Block(768, 256), Block(384, 128)
        self.d2, self.d1 = Block(192, 64), Block(128, 48)
        self.drop = nn.Dropout2d(0.1)
        self.head = nn.Conv2d(48, 5, 1)

    def forward(self, x):
        size = x.shape[-2:]
        e0 = self.stem(x)
        e1 = self.l1(self.pool(e0)); e2 = self.l2(e1); e3 = self.l3(e2); e4 = self.l4(e3)
        up = lambda value, ref: F.interpolate(value, size=ref.shape[-2:], mode='bilinear', align_corners=False)
        d = self.d4(torch.cat([up(e4, e3), e3], 1))
        d = self.d3(torch.cat([up(d, e2), e2], 1))
        d = self.d2(torch.cat([up(d, e1), e1], 1))
        d = self.d1(torch.cat([up(d, e0), e0], 1))
        return F.interpolate(self.head(self.drop(d)), size=size, mode='bilinear', align_corners=False)


class Pairs(Dataset):
    def __init__(self, rows, size, train=False):
        self.rows, self.size, self.train = rows, size, train

    def __len__(self):
        return len(self.rows)

    def image(self, row):
        with Image.open(row['image']) as im:
            return np.array(im.convert('RGB').resize((self.size, self.size), Image.Resampling.BILINEAR), dtype=np.float32) / 255

    def __getitem__(self, i):
        row = self.rows[i]
        x = self.image(row)
        with Image.open(row['mask']) as im:
            if im.mode not in ('L', 'P'):
                raise ValueError('Expected indexed mask')
            y = np.array(im.resize((self.size, self.size), Image.Resampling.NEAREST), dtype=np.int64)
        if set(np.unique(y)) - {0, 1, 2, 3, 4, 255}:
            raise ValueError('Invalid target ID')
        if self.train:
            if random.random() < 0.5:
                x, y = x[:, ::-1], y[:, ::-1]
            x = np.clip(x * random.uniform(.85, 1.15), 0, 1)
        return normalized_tensor(x), torch.from_numpy(y.copy())


def normalized_tensor(x):
    x = (x - np.array([.485, .456, .406], np.float32)) / np.array([.229, .224, .225], np.float32)
    return torch.from_numpy(x.transpose(2, 0, 1).copy())


@torch.inference_mode()
def evaluate(model, loader, device):
    model.eval()
    matrix = np.zeros((5, 5), dtype=np.int64)
    for x, y in loader:
        with torch.autocast('cuda', enabled=device.type == 'cuda'):
            logits = model(x.to(device, non_blocking=True))
        # Evaluate at original indexed-mask resolution, preserving sparse targets.
        for i, target in enumerate(y):
            pred = F.interpolate(logits[i:i+1].float(), size=target.shape, mode='bilinear', align_corners=False).argmax(1)[0].cpu().numpy()
            target = target.numpy()
            valid = target != 255
            matrix += np.bincount(target[valid] * 5 + pred[valid], minlength=25).reshape(5, 5)
    return summarize_confusion(matrix, CLASS_NAMES), matrix


class Validation(Pairs):
    def __getitem__(self, i):
        with Image.open(self.rows[i]['mask']) as im:
            if im.mode not in ('L', 'P'):
                raise ValueError('Expected indexed mask')
            target = torch.from_numpy(np.array(im, dtype=np.int64))
        return normalized_tensor(self.image(self.rows[i])), target


def collate(batch):
    return torch.stack([x for x, _ in batch]), [y for _, y in batch]


def masked_dice(logits, target):
    valid = target != 255
    probs = logits.float().softmax(1) * valid[:, None]
    truth = F.one_hot(target.clamp(max=4), 5).permute(0, 3, 1, 2).float() * valid[:, None]
    intersect = (probs * truth).sum((0, 2, 3))
    denominator = probs.sum((0, 2, 3)) + truth.sum((0, 2, 3))
    present = truth.sum((0, 2, 3)) > 0
    return (1 - (2 * intersect + 1) / (denominator + 1))[present].mean()


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--corpus', type=Path, required=True)
    p.add_argument('--gates', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--epochs', type=int, default=40)
    p.add_argument('--batch-size', type=int, default=2)
    p.add_argument('--size', type=int, default=320)
    p.add_argument('--workers', type=int, default=0, help='parallel training data-loader processes')
    p.add_argument('--resume', action='store_true')
    p.add_argument('--stop-after-epoch', type=int, help='finish a checkpointed chunk while retaining the full epoch/LR budget')
    args = p.parse_args()
    if args.workers < 0:
        p.error('--workers must be nonnegative')
    if not torch.cuda.is_available():
        raise RuntimeError('CUDA unavailable; refusing accidental slow CPU run')
    torch.set_num_threads(4)
    seed = 20261110
    random.seed(seed); np.random.seed(seed); torch.manual_seed(seed); torch.cuda.manual_seed_all(seed)
    records = load_records(args.corpus / 'manifest.csv', args.corpus, args.corpus / 'splits.json', args.corpus / 'sources.json')
    support = audit_class_support(records)
    gates = load_score_gates(args.gates, CLASS_NAMES)
    print('PREFLIGHT PASS', {s: len(r) for s, r in records.items()}, 'support', support, flush=True)
    args.output.mkdir(parents=True, exist_ok=args.resume)
    identity = dict(configuration={k: str(v) if isinstance(v, Path) else v for k, v in vars(args).items() if k not in ('resume', 'stop_after_epoch')},
                    hashes={name: sha256_file(args.corpus / name) for name in ['manifest.csv', 'sources.json', 'splits.json']},
                    gates_sha256=sha256_file(args.gates), trainer_sha256=sha256_file(Path(__file__)))
    data_hashes = {path: sha256_file(Path(path)) for path in sorted({r[k] for rs in records.values() for r in rs for k in ['image', 'mask']})}
    identity['data_sha256'] = hashlib.sha256(json.dumps(data_hashes, sort_keys=True).encode()).hexdigest()
    config = dict(identity=identity, model='UNetR34', preprocessing='RGB /255, ImageNet mean/std; bilinear input; nearest masks',
                  class_names=CLASS_NAMES, ignore_id=255, seed=seed, gates=gates, class_support=support,
                  loss='inverse-sqrt weighted cross-entropy + masked Dice',
                  learning_rate=dict(encoder=.00025, decoder=.0005, schedule='one-epoch warmup, cosine decay'),
                  torch_version=torch.__version__, torchvision_version=torchvision.__version__, test_predictions_inspected=False)
    (args.output / 'run-config.json').write_text(json.dumps(config, indent=2) + '\n')
    (args.output / 'data-hashes.json').write_text(json.dumps(data_hashes, indent=2) + '\n')
    device = torch.device('cuda')
    model = UNetR34(pretrained=not args.resume).to(device)
    encoder = [p for module in (model.stem, model.l1, model.l2, model.l3, model.l4) for p in module.parameters()]
    encoder_ids = {id(p) for p in encoder}
    decoder = [p for p in model.parameters() if id(p) not in encoder_ids]
    opt = torch.optim.AdamW([dict(params=encoder, lr=.00025), dict(params=decoder, lr=.0005)], weight_decay=.0001)
    scaler = torch.amp.GradScaler('cuda')
    ema = copy.deepcopy(model).eval()
    for param in ema.parameters():
        param.requires_grad_(False)
    weights = np.clip(np.sqrt(sum(support['train']) / (5 * np.array(support['train']))), .25, 20)
    criterion = nn.CrossEntropyLoss(weight=torch.tensor(weights, dtype=torch.float32, device=device), ignore_index=255)
    sources = Counter(r['source_id'] for r in records['train'])
    sampling = [sources[r['source_id']] ** -.5 for r in records['train']]
    generator = torch.Generator().manual_seed(seed)
    sampler = WeightedRandomSampler(sampling, len(sampling), replacement=True, generator=generator)
    worker_rng = torch.Generator()
    train = DataLoader(Pairs(records['train'], args.size, True), batch_size=args.batch_size,
                       sampler=sampler, num_workers=args.workers, pin_memory=True,
                       generator=worker_rng)
    validation = DataLoader(Validation(records['validation'], args.size), batch_size=args.batch_size,
                            collate_fn=collate, pin_memory=True)
    start, best, history = 0, -1., []
    if args.resume:
        saved = torch.load(args.output / 'last.pt', map_location='cpu', weights_only=False)
        if saved['identity'] != identity:
            raise ValueError('Resume identity differs')
        if not (args.output / 'best.pt').is_file():
            raise ValueError('Missing original best.pt')
        model.load_state_dict(saved['model']); ema.load_state_dict(saved['ema']); opt.load_state_dict(saved['optimizer']); scaler.load_state_dict(saved['scaler'])
        start, best, history = saved['epoch'], saved['best'], saved['history']
        random.setstate(saved['python_rng']); np.random.set_state(saved['numpy_rng']); torch.set_rng_state(saved['torch_rng'])
        torch.cuda.set_rng_state_all(saved['cuda_rng']); generator.set_state(saved['sampler_rng'])
    last_epoch = min(args.epochs, args.stop_after_epoch or args.epochs)
    if last_epoch <= start:
        raise ValueError('Chunk must end after the resumed epoch')
    for epoch in range(start, last_epoch):
        started = time.monotonic()
        worker_rng.manual_seed(seed + epoch)
        model.train(); loss_sum = 0.; used = 0
        for step, (x, y) in enumerate(train):
            global_step = epoch * len(train) + step
            warmup = len(train)
            total = args.epochs * len(train)
            factor = ((global_step + 1) / warmup if global_step < warmup else
                      .5 * (1 + math.cos(math.pi * (global_step - warmup) / max(total - warmup, 1))))
            for group, base_lr in zip(opt.param_groups, [.00025, .0005]):
                group['lr'] = base_lr * max(factor, .01)
            if not (y != 255).any():
                continue
            x, y = x.to(device, non_blocking=True), y.to(device, non_blocking=True)
            opt.zero_grad(set_to_none=True)
            with torch.autocast('cuda', dtype=torch.float16):
                logits = model(x)
            loss = criterion(logits.float(), y) + masked_dice(logits, y)
            if not torch.isfinite(loss):
                raise RuntimeError('Nonfinite loss')
            scaler.scale(loss).backward(); scaler.unscale_(opt); nn.utils.clip_grad_norm_(model.parameters(), 2.)
            scaler.step(opt); scaler.update()
            with torch.no_grad():
                current = model.state_dict()
                decay = min(.999, (1 + global_step) / (10 + global_step))
                for key, value in ema.state_dict().items():
                    source = current[key]
                    if value.is_floating_point():
                        value.mul_(decay).add_(source, alpha=1-decay)
                    else:
                        value.copy_(source)
            loss_sum += loss.item(); used += 1
            if step % 100 == 0:
                print(f'epoch {epoch+1} batch {step}/{len(train)} loss {loss.item():.4f}', flush=True)
        metrics, matrix = evaluate(ema, validation, device)
        passed = (metrics['miou'] is not None and metrics['miou'] >= gates['minimum_miou']
                  and all(metrics['per_class'][name]['support_pixels'] > 0
                          and metrics['per_class'][name]['iou'] is not None
                          and metrics['per_class'][name]['iou'] >= gates['minimum_per_class_iou'][name]
                          for name in CLASS_NAMES))
        row = dict(epoch=epoch+1, seconds=round(time.monotonic() - started, 1),
                   train_loss=loss_sum/max(used, 1), metrics=metrics, confusion=matrix.tolist(),
                   validation_gate_pass=passed, test_predictions_inspected=False)
        history.append(row)
        if metrics['miou'] > best:
            best = metrics['miou']
            torch.save(dict(state_dict=ema.state_dict(), epoch=epoch+1, model='UNetR34', class_names=CLASS_NAMES,
                            image_size=args.size, preprocessing=config['preprocessing']), args.output / 'best.pt')
            (args.output / 'validation-metrics.json').write_text(json.dumps(
                dict(result='VALIDATION_ONLY', **row), indent=2) + '\n')
        state = dict(model=model.state_dict(), ema=ema.state_dict(), optimizer=opt.state_dict(), scaler=scaler.state_dict(),
                     epoch=epoch+1, best=best, history=history, identity=identity, python_rng=random.getstate(), numpy_rng=np.random.get_state(),
                     torch_rng=torch.get_rng_state(), cuda_rng=torch.cuda.get_rng_state_all(), sampler_rng=generator.get_state())
        torch.save(state, args.output / 'last.tmp.pt'); (args.output / 'last.tmp.pt').replace(args.output / 'last.pt')
        (args.output / 'history.json').write_text(json.dumps(history, indent=2) + '\n')
        print(json.dumps(row), flush=True)
    print(f'Completed epoch {last_epoch}/{args.epochs}. Test predictions remain uninspected.', flush=True)


if __name__ == '__main__':
    main()
