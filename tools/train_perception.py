#!/usr/bin/env python3
"""Reproducible semantic segmentation training and held-out evaluation.

Input CSV columns: image,mask,source_id,terrain_id,split. Paths are relative to
--manifest-root (default: the CSV parent) or absolute. Class IDs come from the
supplied class manifest; 255 is always ignore.
"""
from __future__ import annotations
import argparse
import csv
import hashlib
import json
import random
import subprocess
import sys
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import numpy as np
from PIL import Image
from perception_metrics import summarize_confusion

ROOT = Path(__file__).resolve().parents[1]
CLASS_NAMES = ["regolith", "bedrock", "rock", "crater", "shadow"]
IGNORE_ID = 255


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def load_records(manifest: Path, manifest_root: Path, splits_path: Path, sources_path: Path) -> dict[str, list[dict[str, str]]]:
    split_policy = json.loads(splits_path.read_text(encoding="utf-8"))
    sources = json.loads(sources_path.read_text(encoding="utf-8"))
    approved = {row["source_id"]: row for row in sources["selected_sources"]}
    allowed_groups: dict[tuple[str, str], str] = {}
    namespaces: dict[tuple[str, str], str] = {}
    for row in split_policy.get("scene_splits", []):
        source_id = row["source_id"]
        for split in ("train", "validation", "test"):
            for terrain in row.get(split, []):
                key = (row.get("group_namespace", source_id), terrain)
                if key in namespaces and namespaces[key] != split:
                    raise ValueError(f"split policy itself leaks {key}")
                namespaces[key] = split
                allowed_groups[(source_id, terrain)] = split
    train_only = set(split_policy.get("train_only_sources", []))
    evaluation_only = set(split_policy.get("evaluation_only_sources", []))
    records: dict[str, list[dict[str, str]]] = {"train": [], "validation": [], "test": []}
    seen_pairs: set[tuple[str, str, str]] = set()
    with manifest.open("r", encoding="utf-8-sig", newline="") as stream:
        reader = csv.DictReader(stream)
        required = {"image", "mask", "source_id", "terrain_id", "split"}
        if not reader.fieldnames or not required.issubset(reader.fieldnames):
            raise ValueError(f"manifest columns must include {sorted(required)}")
        for line_no, raw in enumerate(reader, start=2):
            row = {key: (value or "").strip() for key, value in raw.items()}
            source_id, terrain, split = row["source_id"], row["terrain_id"], row["split"]
            if source_id not in approved:
                raise ValueError(f"line {line_no}: unapproved source {source_id!r}")
            if split not in records:
                raise ValueError(f"line {line_no}: invalid split {split!r}")
            if source_id in train_only and split != "train":
                raise ValueError(f"line {line_no}: training-only source assigned to {split}")
            if source_id in evaluation_only and split == "train":
                raise ValueError(f"line {line_no}: evaluation-only source assigned to train")
            if source_id not in train_only:
                expected = allowed_groups.get((source_id, terrain))
                if expected != split:
                    raise ValueError(f"line {line_no}: {source_id}/{terrain} belongs to {expected!r}, not {split!r}")
            pair = (source_id, terrain, split)
            if pair in seen_pairs and source_id not in train_only and allowed_groups.get((source_id, terrain)) != split:
                raise ValueError(f"line {line_no}: scene split mismatch")
            seen_pairs.add(pair)
            for field in ("image", "mask"):
                path = Path(row[field])
                if not path.is_absolute():
                    path = manifest_root / path
                path = path.resolve()
                if not path.is_file():
                    raise FileNotFoundError(f"line {line_no}: missing {field}: {path}")
                row[field] = str(path)
            records[split].append(row)
    for split, rows in records.items():
        if not rows:
            raise ValueError(f"manifest has no {split} records")
    if not any(row["source_id"] == "polar-sim-labels" for row in records["validation"] + records["test"]):
        raise ValueError("validation and test must include held-out POLAR-Sim terrain scenes")
    return records


def audit_class_support(records: dict[str, list[dict[str, str]]], class_count: int = 5) -> dict[str, list[int]]:
    """Count target pixels in each split and reject a split missing any class."""
    support = {split: [0] * class_count for split in ("train", "validation", "test")}
    for split, rows in records.items():
        for row in rows:
            with Image.open(row["mask"]) as image:
                if image.mode not in ("L", "P"):
                    raise ValueError(f"mask must be indexed grayscale/palette PNG: {row['mask']} ({image.mode})")
                mask = np.asarray(image, dtype=np.uint8)
            values, counts = np.unique(mask, return_counts=True)
            for value, count in zip(values.tolist(), counts.tolist()):
                if 0 <= value < class_count:
                    support[split][value] += int(count)
                elif value != IGNORE_ID:
                    raise ValueError(f"mask contains unsupported class ID {value}: {row['mask']}")
    missing = {
        split: [class_id for class_id, count in enumerate(counts) if count == 0]
        for split, counts in support.items()
    }
    missing = {split: ids for split, ids in missing.items() if ids}
    if missing:
        raise ValueError(
            f"qualification requires pixel support for every class in train/validation/test; "
            f"missing class IDs by split: {missing}; support: {support}")
    return support


def load_score_gates(path: Path, class_names: list[str]) -> dict[str, Any]:
    """Load the frozen numeric gates required before any held-out test pass."""
    document = json.loads(path.read_text(encoding="utf-8"))
    if document.get("schema_version") != 1 or not isinstance(document.get("gate_id"), str) or not document["gate_id"].strip():
        raise ValueError("gates require schema_version=1 and a non-empty gate_id")
    minimum_miou = document.get("minimum_miou")
    per_class = document.get("minimum_per_class_iou")
    if isinstance(minimum_miou, bool) or not isinstance(minimum_miou, (int, float)) or not 0.0 <= minimum_miou <= 1.0:
        raise ValueError("minimum_miou must be numeric in [0, 1]")
    if not isinstance(per_class, dict) or set(per_class) != set(class_names):
        raise ValueError("minimum_per_class_iou must define exactly the classes in the supplied class manifest")
    checked = {}
    for name in class_names:
        value = per_class[name]
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not 0.0 <= value <= 1.0:
            raise ValueError(f"minimum IoU for {name} must be numeric in [0, 1]")
        checked[name] = float(value)
    scope = document.get("scope", "unspecified")
    if not isinstance(scope, str) or not scope.strip():
        raise ValueError("gate scope must be a non-empty string when provided")
    return {"schema_version": 1, "gate_id": document["gate_id"], "scope": scope,
            "minimum_miou": float(minimum_miou), "minimum_per_class_iou": checked}


def _torch_components():
    try:
        import torch
        from torch import nn
        from torch.utils.data import DataLoader, Dataset
    except ImportError as exc:
        raise RuntimeError("PyTorch is required; install the project-selected torch build before training") from exc

    class ConvBlock(nn.Module):
        def __init__(self, incoming: int, outgoing: int):
            super().__init__()
            self.layers = nn.Sequential(
                nn.Conv2d(incoming, outgoing, 3, padding=1, bias=False),
                nn.BatchNorm2d(outgoing), nn.ReLU(inplace=True),
                nn.Conv2d(outgoing, outgoing, 3, padding=1, bias=False),
                nn.BatchNorm2d(outgoing), nn.ReLU(inplace=True),
            )
        def forward(self, x):
            return self.layers(x)

    class SmallUNet(nn.Module):
        def __init__(self, class_count: int = 5, width: int = 16):
            super().__init__()
            self.enc1 = ConvBlock(3, width)
            self.enc2 = ConvBlock(width, width * 2)
            self.enc3 = ConvBlock(width * 2, width * 4)
            self.pool = nn.MaxPool2d(2)
            self.up2 = nn.ConvTranspose2d(width * 4, width * 2, 2, stride=2)
            self.dec2 = ConvBlock(width * 4, width * 2)
            self.up1 = nn.ConvTranspose2d(width * 2, width, 2, stride=2)
            self.dec1 = ConvBlock(width * 2, width)
            self.head = nn.Conv2d(width, class_count, 1)
        def forward(self, x):
            a = self.enc1(x)
            b = self.enc2(self.pool(a))
            c = self.enc3(self.pool(b))
            d = self.dec2(__import__("torch").cat([self.up2(c), b], dim=1))
            e = self.dec1(__import__("torch").cat([self.up1(d), a], dim=1))
            return self.head(e)

    class SegmentationDataset(Dataset):
        def __init__(self, rows: list[dict[str, str]], size: int, class_count: int):
            self.rows, self.size, self.class_count = rows, size, class_count
        def __len__(self):
            return len(self.rows)
        def __getitem__(self, index):
            row = self.rows[index]
            image = Image.open(row["image"]).convert("RGB").resize((self.size, self.size), getattr(Image, "Resampling", Image).BILINEAR)
            mask = Image.open(row["mask"])
            if mask.mode not in ("L", "P"):
                raise ValueError(f"mask must be indexed grayscale/palette PNG: {row['mask']} ({mask.mode})")
            mask = mask.resize((self.size, self.size), getattr(Image, "Resampling", Image).NEAREST)
            x = np.asarray(image, dtype=np.float32).transpose(2, 0, 1) / 255.0
            y = np.asarray(mask, dtype=np.int64)
            invalid = ~(((y >= 0) & (y < self.class_count)) | (y == IGNORE_ID))
            if np.any(invalid):
                raise ValueError(f"mask contains IDs outside 0..{self.class_count - 1} and 255: {row['mask']}")
            return torch.from_numpy(x), torch.from_numpy(y)

    return torch, nn, DataLoader, Dataset, SmallUNet, SegmentationDataset


def _evaluate(model, loader, torch, device, class_count: int) -> tuple[float, np.ndarray]:
    model.eval()
    total_loss = 0.0
    pixels = 0
    matrix = np.zeros((class_count, class_count), dtype=np.int64)
    criterion = torch.nn.CrossEntropyLoss(ignore_index=IGNORE_ID, reduction="sum")
    with torch.no_grad():
        for images, targets in loader:
            images, targets = images.to(device), targets.to(device)
            logits = model(images)
            total_loss += float(criterion(logits, targets).item())
            pixels += int((targets != IGNORE_ID).sum().item())
            pred = logits.argmax(1).cpu().numpy().astype(np.int64)
            truth = targets.cpu().numpy().astype(np.int64)
            valid = truth != IGNORE_ID
            encoded = truth[valid] * class_count + pred[valid]
            matrix += np.bincount(encoded, minlength=class_count * class_count).reshape(class_count, class_count)
    return (total_loss / max(pixels, 1)), matrix


def git_revision() -> str | None:
    try:
        return subprocess.check_output(["git", "-C", str(ROOT), "rev-parse", "HEAD"], text=True, stderr=subprocess.DEVNULL).strip()
    except (OSError, subprocess.CalledProcessError):
        return None


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest", type=Path, required=True, help="CSV of reviewed image/mask/source/terrain/split records")
    parser.add_argument("--manifest-root", type=Path, help="base for relative paths; defaults to manifest parent")
    parser.add_argument("--classes", type=Path, default=ROOT / "config/perception/classes.json")
    parser.add_argument("--sources", type=Path, default=ROOT / "config/perception/sources.json")
    parser.add_argument("--splits", type=Path, default=ROOT / "config/perception/splits.json")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--gates", type=Path, required=True, help="frozen JSON mIoU and per-class IoU thresholds; required before test evaluation")
    parser.add_argument("--epochs", type=int, default=30)
    parser.add_argument("--batch-size", type=int, default=2)
    parser.add_argument("--image-size", type=int, default=256)
    parser.add_argument("--learning-rate", type=float, default=0.001)
    parser.add_argument("--seed", type=int, default=20261104)
    parser.add_argument("--device", choices=("auto", "cpu", "cuda"), default="auto")
    parser.add_argument("--num-workers", type=int, default=0)
    parser.add_argument("--cpu-threads", type=int, default=4)
    parser.add_argument("--class-weighting", choices=("none", "inverse_sqrt"), default="none")
    parser.add_argument("--resume", type=Path, help="resume last.pt from an interrupted run with the same data and gates")
    parser.add_argument("--skip-test", action="store_true", help="development run: save validation results without inspecting held-out test predictions")
    args = parser.parse_args()
    if args.epochs < 1 or args.batch_size < 1 or args.image_size < 32 or args.image_size % 4:
        parser.error("epochs/batch-size must be positive; image-size must be >=32 and divisible by 4")
    try:
        classes_doc = json.loads(args.classes.read_text(encoding="utf-8"))
        names = [row["name"] for row in classes_doc["classes"]]
        if (not names or [row.get("id") for row in classes_doc["classes"]] != list(range(len(names)))
                or len(set(names)) != len(names) or classes_doc.get("ignore_id") != IGNORE_ID):
            raise ValueError("class manifest must define unique names with contiguous IDs starting at zero and ignore_id 255")
        manifest = args.manifest.resolve()
        root = (args.manifest_root or manifest.parent).resolve()
        records = load_records(manifest, root, args.splits, args.sources)
        support = audit_class_support(records, len(names))
        gates = load_score_gates(args.gates, names)
        torch, _, DataLoader, _, Model, Dataset = _torch_components()
    except (OSError, json.JSONDecodeError, KeyError, ValueError, RuntimeError) as exc:
        print(f"FAIL: {exc}", file=sys.stderr)
        return 2
    if args.device == "cuda" and not torch.cuda.is_available():
        print("FAIL: CUDA requested but unavailable", file=sys.stderr)
        return 2
    device = torch.device("cuda" if args.device == "cuda" or (args.device == "auto" and torch.cuda.is_available()) else "cpu")
    if args.cpu_threads < 1:
        parser.error("cpu-threads must be positive")
    torch.set_num_threads(args.cpu_threads)
    random.seed(args.seed)
    np.random.seed(args.seed)
    torch.manual_seed(args.seed)
    if torch.cuda.is_available():
        torch.cuda.manual_seed_all(args.seed)
    torch.use_deterministic_algorithms(True, warn_only=True)
    args.output.mkdir(parents=True, exist_ok=bool(args.resume))
    generator = torch.Generator().manual_seed(args.seed)
    train_loader = DataLoader(Dataset(records["train"], args.image_size, len(names)), batch_size=args.batch_size, shuffle=True, num_workers=args.num_workers, generator=generator)
    val_loader = DataLoader(Dataset(records["validation"], args.image_size, len(names)), batch_size=args.batch_size, shuffle=False, num_workers=args.num_workers)
    test_loader = DataLoader(Dataset(records["test"], args.image_size, len(names)), batch_size=args.batch_size, shuffle=False, num_workers=args.num_workers)
    model = Model(len(names)).to(device)
    optimizer = torch.optim.AdamW(model.parameters(), lr=args.learning_rate)
    class_weights = None
    if args.class_weighting == "inverse_sqrt":
        counts = np.asarray(support["train"], dtype=np.float64)
        weights = np.clip(np.sqrt(counts.sum() / (len(names) * counts)), 0.25, 20.0)
        class_weights = torch.tensor(weights, dtype=torch.float32, device=device)
    criterion = torch.nn.CrossEntropyLoss(ignore_index=IGNORE_ID, weight=class_weights)
    best_miou = -1.0
    history: list[dict[str, Any]] = []
    identity = {"manifest_sha256": sha256_file(manifest), "classes_sha256": sha256_file(args.classes),
                "class_names": names, "gates_sha256": sha256_file(args.gates),
                "image_size": args.image_size, "batch_size": args.batch_size,
                "seed": args.seed, "class_weighting": args.class_weighting,
                "learning_rate": args.learning_rate,
                "split_policy_sha256": sha256_file(args.splits),
                "sources_sha256": sha256_file(args.sources),
                "trainer_sha256": sha256_file(Path(__file__))}
    initial_data_hashes = {path: sha256_file(Path(path)) for path in sorted({
        row[field] for values in records.values() for row in values for field in ("image", "mask")})}
    identity["data_content_sha256"] = hashlib.sha256(json.dumps(initial_data_hashes, sort_keys=True).encode()).hexdigest()
    first_epoch = 1
    if args.resume:
        saved = torch.load(args.resume, map_location="cpu", weights_only=True)
        if saved.get("run_identity") != identity:
            raise ValueError("resume checkpoint data, gates or training configuration differ")
        if not (args.output / "best.pt").is_file():
            raise ValueError("resume output must retain the original best.pt")
        model.load_state_dict(saved["state_dict"])
        optimizer.load_state_dict(saved["optimizer_state_dict"])
        history = saved["history"]
        best_miou = saved["best_validation_miou"]
        first_epoch = saved["epoch"] + 1
        torch.set_rng_state(saved["torch_rng_state"])
        generator.set_state(saved["loader_rng_state"])
        if device.type == "cuda" and saved.get("cuda_rng_states"):
            torch.cuda.set_rng_state_all(saved["cuda_rng_states"])
    (args.output / "run-config.json").write_text(json.dumps({**identity, "device": str(device),
        "epochs": args.epochs, "resumed_from": str(args.resume) if args.resume else None}, indent=2)+"\n")
    for epoch in range(first_epoch, args.epochs + 1):
        model.train()
        train_loss_sum, train_pixels = 0.0, 0
        for images, targets in train_loader:
            images, targets = images.to(device), targets.to(device)
            if not bool((targets != IGNORE_ID).any()):
                continue
            optimizer.zero_grad(set_to_none=True)
            logits = model(images)
            loss = criterion(logits, targets)
            loss.backward()
            optimizer.step()
            valid_count = int((targets != IGNORE_ID).sum().item())
            train_loss_sum += float(loss.item()) * valid_count
            train_pixels += valid_count
        val_loss, val_matrix = _evaluate(model, val_loader, torch, device, len(names))
        val_metrics = summarize_confusion(val_matrix, names)
        row = {"epoch": epoch, "train_loss": train_loss_sum / max(train_pixels, 1), "validation_loss": val_loss, "validation_miou": val_metrics["miou"], "validation_supported_classes": val_metrics["miou_supported_class_count"]}
        row["validation_per_class"] = val_metrics["per_class"]
        history.append(row)
        # Persist progress every epoch so a long run remains inspectable even
        # when the controlling terminal is disconnected.
        (args.output / "history.json").write_text(json.dumps(history, indent=2) + "\n", encoding="utf-8")
        score = val_metrics["miou"]
        if score is not None and score > best_miou:
            best_miou = score
            torch.save({"state_dict": model.state_dict(), "epoch": epoch, "class_names": names, "image_size": args.image_size}, args.output / "best.pt")
        torch.save({"state_dict": model.state_dict(), "optimizer_state_dict": optimizer.state_dict(),
                    "epoch": epoch, "class_names": names, "image_size": args.image_size,
                    "history": history, "best_validation_miou": best_miou,
                    "run_identity": identity, "torch_rng_state": torch.get_rng_state(),
                    "loader_rng_state": generator.get_state(),
                    "cuda_rng_states": torch.cuda.get_rng_state_all() if device.type == "cuda" else []},
                   args.output / "last.tmp.pt")
        (args.output / "last.tmp.pt").replace(args.output / "last.pt")
        print(json.dumps(row, sort_keys=True), flush=True)
    if best_miou < 0:
        print("FAIL: validation produced no supported class IoU; refusing test evaluation", file=sys.stderr)
        return 2
    if (any(sha256_file(Path(path)) != digest for path, digest in initial_data_hashes.items())
            or sha256_file(args.gates) != identity["gates_sha256"]
            or sha256_file(args.sources) != identity["sources_sha256"]
            or sha256_file(args.splits) != identity["split_policy_sha256"]):
        print("FAIL: frozen data or configuration changed during training; refusing test evaluation", file=sys.stderr)
        return 2
    checkpoint = torch.load(args.output / "best.pt", map_location=device, weights_only=True)
    model.load_state_dict(checkpoint["state_dict"])
    if args.skip_test:
        validation_loss, validation_matrix = _evaluate(model, val_loader, torch, device, len(names))
        validation_metrics = summarize_confusion(validation_matrix, names)
        validation_metrics.update(loss=validation_loss, checkpoint_epoch=checkpoint["epoch"])
        validation_gate_pass = (validation_metrics["miou"] is not None
            and validation_metrics["miou"] >= gates["minimum_miou"]
            and all(validation_metrics["per_class"][name]["iou"] is not None
                and validation_metrics["per_class"][name]["support_pixels"] > 0
                and validation_metrics["per_class"][name]["iou"] >= gates["minimum_per_class_iou"][name]
                for name in names))
        document = {"result": "VALIDATION_ONLY", "test_predictions_inspected": False,
                    "validation_gate_pass": validation_gate_pass, "gates": gates,
                    "run_identity": identity, "checkpoint_sha256": sha256_file(args.output/"best.pt"),
                    "validation_metrics": validation_metrics, "history": history}
        (args.output/"validation-metrics.json").write_text(json.dumps(document,indent=2)+"\n")
        print(json.dumps({"result":"VALIDATION_ONLY", "validation_miou":validation_metrics["miou"],
                          "validation_gate_pass":validation_gate_pass}),flush=True)
        return 0
    test_loss, test_matrix = _evaluate(model, test_loader, torch, device, len(names))
    test_metrics = summarize_confusion(test_matrix, names)
    test_metrics.update({"loss": test_loss, "checkpoint_epoch": checkpoint["epoch"], "records": len(records["test"])})
    test_metrics["all_target_classes_supported"] = all(test_metrics["per_class"][name]["support_pixels"] > 0 for name in names)
    class_gate_results = {
        name: {
            "minimum_iou": gates["minimum_per_class_iou"][name],
            "actual_iou": test_metrics["per_class"][name]["iou"],
            "pass": test_metrics["per_class"][name]["iou"] is not None
                    and test_metrics["per_class"][name]["support_pixels"] > 0
                    and test_metrics["per_class"][name]["iou"] >= gates["minimum_per_class_iou"][name],
        }
        for name in names
    }
    gate_result = {
        "gate_id": gates["gate_id"],
        "scope": gates["scope"],
        "minimum_miou": gates["minimum_miou"],
        "actual_miou": test_metrics["miou"],
        "miou_pass": test_metrics["miou"] is not None and test_metrics["miou"] >= gates["minimum_miou"],
        "per_class": class_gate_results,
    }
    gate_result["pass"] = gate_result["miou_pass"] and all(row["pass"] for row in class_gate_results.values())
    provenance = {
        "task_id": "P5-02", "created_utc": datetime.now(timezone.utc).isoformat(),
        "git_revision": git_revision(), "seed": args.seed, "device": str(device),
        "torch_version": torch.__version__, "cuda_version": torch.version.cuda,
        "class_names": names, "ignore_id": IGNORE_ID,
        "model": "small_unet_width_16", "image_size": args.image_size,
        "epochs_requested": args.epochs, "batch_size": args.batch_size,
        "learning_rate": args.learning_rate, "manifest": str(manifest),
        "class_weighting": args.class_weighting,
        "run_identity": identity,
        "resumed_from": str(args.resume) if args.resume else None,
        "class_weights": class_weights.cpu().tolist() if class_weights is not None else None,
        "cpu_threads": args.cpu_threads,
        "manifest_sha256": sha256_file(manifest), "split_policy_sha256": sha256_file(args.splits),
        "gates": gates, "gates_sha256": sha256_file(args.gates),
        "dataset_records": {key: len(value) for key, value in records.items()},
        "source_records_by_split": {
            split: {source_id: sum(row["source_id"] == source_id for row in rows)
                    for source_id in sorted({row["source_id"] for row in rows})}
            for split, rows in records.items()
        },
        "class_pixel_support": support,
        "data_sha256": initial_data_hashes,
        "best_epoch": checkpoint["epoch"], "checkpoint_sha256": sha256_file(args.output / "best.pt"), "history": history,
        "test_metrics": test_metrics,
        "qualification_gate": gate_result,
    }
    (args.output / "metrics.json").write_text(json.dumps(provenance, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps({"result": "PASS" if gate_result["pass"] else "FAIL", "output": str(args.output), "test_miou": test_metrics["miou"], "test_all_target_classes_supported": test_metrics["all_target_classes_supported"], "gate_id": gates["gate_id"]}, sort_keys=True))
    return 0 if gate_result["pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
