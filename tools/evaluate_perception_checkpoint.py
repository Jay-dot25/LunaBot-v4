#!/usr/bin/env python3
"""Score a validation-selected P5-02 checkpoint once on its held-out split."""
from __future__ import annotations

import argparse
import hashlib
import json
from datetime import datetime, timezone
from pathlib import Path
import random
import sys

import numpy as np

from perception_metrics import summarize_confusion
from train_perception import (
    CLASS_NAMES,
    IGNORE_ID,
    ROOT,
    _evaluate,
    _torch_components,
    audit_class_support,
    git_revision,
    load_records,
    load_score_gates,
    sha256_file,
)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True, help="completed training run directory")
    parser.add_argument("--gates", type=Path, required=True)
    parser.add_argument("--sources", type=Path, required=True)
    parser.add_argument("--splits", type=Path, required=True)
    parser.add_argument("--device", choices=("cpu", "cuda"), default="cuda")
    parser.add_argument("--cpu-threads", type=int, default=4)
    args = parser.parse_args()

    manifest = args.manifest.resolve()
    run_config_path = args.output / "run-config.json"
    validation_path = args.output / "validation-metrics.json"
    checkpoint_path = args.output / "best.pt"
    last_path = args.output / "last.pt"
    try:
        run_config = json.loads(run_config_path.read_text(encoding="utf-8"))
        validation = json.loads(validation_path.read_text(encoding="utf-8"))
        if validation.get("result") != "VALIDATION_ONLY" or validation.get("test_predictions_inspected") is not False:
            raise ValueError("expected a validation-only record with no earlier test scoring")
        checkpoint = __import__("torch").load(checkpoint_path, map_location="cpu", weights_only=True)
        last = __import__("torch").load(last_path, map_location="cpu", weights_only=True)
        identity = last.get("run_identity")
        if not isinstance(identity, dict) or checkpoint.get("epoch") != validation.get("validation_metrics", {}).get("checkpoint_epoch"):
            raise ValueError("selected checkpoint does not match the validation-selected epoch")
        if validation.get("run_identity") != identity:
            raise ValueError("validation and checkpoint run identities differ")
        if any(run_config.get(key) != value for key, value in identity.items()):
            raise ValueError("run configuration and checkpoint identities differ")
        if any(sha256_file(path) != expected for path, expected in (
            (manifest, identity["manifest_sha256"]),
            (args.gates, identity["gates_sha256"]),
            (args.sources, identity["sources_sha256"]),
            (args.splits, identity["split_policy_sha256"]),
        )):
            raise ValueError("manifest or frozen configuration changed since training")
        records = load_records(manifest, manifest.parent, args.splits, args.sources)
        names = CLASS_NAMES
        gates = load_score_gates(args.gates, names)
        support = audit_class_support(records, len(names))
        data_hashes = {path: sha256_file(Path(path)) for path in sorted({
            row[field] for split_rows in records.values() for row in split_rows for field in ("image", "mask")
        })}
        data_digest = hashlib.sha256(json.dumps(data_hashes, sort_keys=True).encode()).hexdigest()
        if data_digest != identity["data_content_sha256"]:
            raise ValueError("dataset content hashes differ from the completed training run")
        if args.cpu_threads < 1:
            raise ValueError("cpu-threads must be positive")
        torch, _, DataLoader, _, Model, Dataset = _torch_components()
        if args.device == "cuda" and not torch.cuda.is_available():
            raise RuntimeError("CUDA requested but unavailable")
        device = torch.device(args.device)
        torch.set_num_threads(args.cpu_threads)
        random.seed(run_config["seed"])
        np.random.seed(run_config["seed"])
        torch.manual_seed(run_config["seed"])
        if device.type == "cuda":
            torch.cuda.manual_seed_all(run_config["seed"])
        torch.use_deterministic_algorithms(True, warn_only=True)
        loader = DataLoader(
            Dataset(records["test"], run_config["image_size"]),
            batch_size=run_config["batch_size"], shuffle=False, num_workers=0,
        )
        model = Model(len(names)).to(device)
        model.load_state_dict(checkpoint["state_dict"])
        test_loss, test_matrix = _evaluate(model, loader, torch, device, len(names))
        test_metrics = summarize_confusion(test_matrix, names)
        test_metrics.update(loss=test_loss, checkpoint_epoch=checkpoint["epoch"], records=len(records["test"]))
        test_metrics["all_target_classes_supported"] = all(
            test_metrics["per_class"][name]["support_pixels"] > 0 for name in names
        )
    except (OSError, KeyError, TypeError, ValueError, RuntimeError, json.JSONDecodeError) as exc:
        print(f"FAIL: {exc}", file=sys.stderr)
        return 2

    per_class = {
        name: {
            "minimum_iou": gates["minimum_per_class_iou"][name],
            "actual_iou": test_metrics["per_class"][name]["iou"],
            "support_pixels": test_metrics["per_class"][name]["support_pixels"],
            "pass": test_metrics["per_class"][name]["iou"] is not None
                    and test_metrics["per_class"][name]["support_pixels"] > 0
                    and test_metrics["per_class"][name]["iou"] >= gates["minimum_per_class_iou"][name],
        }
        for name in names
    }
    gate_result = {
        "gate_id": gates["gate_id"], "scope": gates["scope"],
        "minimum_miou": gates["minimum_miou"], "actual_miou": test_metrics["miou"],
        "miou_pass": test_metrics["miou"] is not None and test_metrics["miou"] >= gates["minimum_miou"],
        "per_class": per_class,
    }
    gate_result["pass"] = gate_result["miou_pass"] and all(row["pass"] for row in per_class.values())
    provenance = {
        "task_id": "P5-02", "created_utc": datetime.now(timezone.utc).isoformat(),
        "git_revision": git_revision(), "result": "PASS" if gate_result["pass"] else "FAIL",
        "test_predictions_inspected": True, "validation_gate_pass": validation["validation_gate_pass"],
        "training_run_identity": identity, "training_trainer_sha256": identity["trainer_sha256"],
        "evaluation_code_sha256": sha256_file(Path(__file__)),
        "torch_version": torch.__version__, "cuda_version": torch.version.cuda,
        "device": str(device), "seed": run_config["seed"], "class_names": names,
        "ignore_id": IGNORE_ID, "model": "small_unet_width_16",
        "image_size": run_config["image_size"], "batch_size": run_config["batch_size"],
        "epochs_requested": run_config["epochs"], "best_epoch": checkpoint["epoch"],
        "manifest": str(manifest), "manifest_sha256": identity["manifest_sha256"],
        "split_policy_sha256": identity["split_policy_sha256"],
        "sources_sha256": identity["sources_sha256"], "gates": gates,
        "gates_sha256": identity["gates_sha256"], "checkpoint_sha256": sha256_file(checkpoint_path),
        "dataset_records": {split: len(rows) for split, rows in records.items()},
        "source_records_by_split": {
            split: {source: sum(row["source_id"] == source for row in rows)
                    for source in sorted({row["source_id"] for row in rows})}
            for split, rows in records.items()
        },
        "class_pixel_support": support, "data_sha256": data_hashes,
        "validation_metrics": validation["validation_metrics"],
        "test_metrics": test_metrics, "qualification_gate": gate_result,
    }
    out_path = args.output / "metrics.json"
    out_path.write_text(json.dumps(provenance, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps({"result": provenance["result"], "output": str(out_path),
                      "test_miou": test_metrics["miou"], "gate_pass": gate_result["pass"]}, sort_keys=True))
    return 0 if gate_result["pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
