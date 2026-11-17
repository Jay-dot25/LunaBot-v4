"""One final test evaluation of a frozen, validation-selected ResNet34 U-Net."""
import argparse
import hashlib
import json
from collections import defaultdict
from pathlib import Path

from perception_metrics import bootstrap_scene_iou, summarize_confusion
import torch
from torch.utils.data import DataLoader

from train_perception import load_records, load_score_gates, sha256_file, CLASS_NAMES
from train_downloaded_resnet import UNetR34, Validation, collate, evaluate


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--corpus', type=Path, required=True)
    p.add_argument('--gates', type=Path, required=True)
    p.add_argument('--run', type=Path, required=True)
    a = p.parse_args()
    output = a.run / 'test-metrics.json'
    if output.exists():
        raise ValueError('Test result already exists; refusing repeated test scoring')
    config = json.loads((a.run / 'run-config.json').read_text())
    identity = config['identity']
    for name, digest in identity['hashes'].items():
        if sha256_file(a.corpus / name) != digest:
            raise ValueError('Corpus configuration changed')
    if sha256_file(a.gates) != identity['gates_sha256']:
        raise ValueError('Frozen gates changed')
    if sha256_file(Path(__file__).with_name('train_downloaded_resnet.py')) != identity['trainer_sha256']:
        raise ValueError('Training/inference code changed')
    records = load_records(a.corpus / 'manifest.csv', a.corpus, a.corpus / 'splits.json', a.corpus / 'sources.json')
    hashes = {path: sha256_file(Path(path)) for path in sorted({r[k] for rs in records.values() for r in rs for k in ['image', 'mask']})}
    if hashlib.sha256(json.dumps(hashes, sort_keys=True).encode()).hexdigest() != identity['data_sha256']:
        raise ValueError('Corpus content changed')
    gates = load_score_gates(a.gates, CLASS_NAMES)
    checkpoint = torch.load(a.run / 'best.pt', map_location='cpu', weights_only=True)
    last = torch.load(a.run / 'last.pt', map_location='cpu', weights_only=False)
    if last['epoch'] < identity['configuration']['epochs']:
        raise ValueError('Finish the planned training budget before final test scoring')
    selected = json.loads((a.run / 'validation-metrics.json').read_text())
    if checkpoint['epoch'] != selected['epoch']:
        raise ValueError('Checkpoint does not match validation selection')
    if not selected['validation_gate_pass']:
        raise ValueError('Validation gates failed; improve the model before final test scoring')
    device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
    model = UNetR34().to(device)
    model.load_state_dict(checkpoint['state_dict'])
    split_policy = json.loads((a.corpus / 'splits.json').read_text())
    namespaces = {entry['source_id']: entry.get('group_namespace', entry['source_id'])
                  for entry in split_policy['scene_splits']}
    scenes = defaultdict(list)
    for row in records['test']:
        source = row['source_id']
        scenes[namespaces.get(source, source), row['terrain_id']].append(row)
    scene_matrices = {}
    for scene, rows in sorted(scenes.items()):
        loader = DataLoader(Validation(rows, checkpoint['image_size']), batch_size=2, collate_fn=collate)
        _, scene_matrices['/'.join(scene)] = evaluate(model, loader, device)
    matrix = sum(scene_matrices.values())
    metrics = summarize_confusion(matrix, CLASS_NAMES)
    passed = (metrics['miou'] is not None and metrics['miou'] >= gates['minimum_miou']
              and all(metrics['per_class'][n]['support_pixels'] > 0
                      and metrics['per_class'][n]['iou'] is not None
                      and metrics['per_class'][n]['iou'] >= gates['minimum_per_class_iou'][n]
                      for n in CLASS_NAMES))
    doc = dict(result='PASS' if passed else 'FAIL', test_gate_pass=passed, test_predictions_inspected=True,
               metrics=metrics, confusion_rows_true_cols_pred=matrix.tolist(), gates=gates,
               scene_confusions={scene: counts.tolist() for scene, counts in scene_matrices.items()},
               scene_bootstrap=bootstrap_scene_iou(list(scene_matrices.values()), CLASS_NAMES),
               checkpoint_sha256=sha256_file(a.run / 'best.pt'), identity=identity,
               limitation='Inherited evaluation includes previously inspected, sparse held-out scenes; not new independent evidence.')
    output.write_text(json.dumps(doc, indent=2) + '\n')
    print(json.dumps(doc, indent=2))
    return 0 if passed else 1


if __name__ == '__main__':
    raise SystemExit(main())
