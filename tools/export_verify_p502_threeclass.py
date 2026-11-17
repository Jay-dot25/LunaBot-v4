#!/usr/bin/env python3
"""Export P5-02 to TorchScript; verify parity, health, latency, and memory."""
from __future__ import annotations

import argparse
import csv
import hashlib
import json
import shutil
import statistics
import sys
import time
import zipfile
from pathlib import Path

import numpy as np
import psutil
import torch
import torch.nn.functional as F
from PIL import Image
from torch import nn
from torch.nn import functional
import torchvision

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
from p502_threeclass_inference import CLASS_NAMES, UNKNOWN_ID, P502ThreeClassRuntime, sha256_file


class Block(nn.Module):
    def __init__(self, incoming: int, outgoing: int) -> None:
        super().__init__()
        self.f = nn.Sequential(
            nn.Conv2d(incoming, outgoing, 3, padding=1, bias=False),
            nn.BatchNorm2d(outgoing), nn.ReLU(inplace=True),
            nn.Conv2d(outgoing, outgoing, 3, padding=1, bias=False),
            nn.BatchNorm2d(outgoing), nn.ReLU(inplace=True),
        )

    def forward(self, value: torch.Tensor) -> torch.Tensor:
        return self.f(value)


class UNetR34(nn.Module):
    def __init__(self) -> None:
        super().__init__()
        encoder = torchvision.models.resnet34(weights=None)
        self.stem = nn.Sequential(encoder.conv1, encoder.bn1, encoder.relu)
        self.pool = encoder.maxpool
        self.l1, self.l2, self.l3, self.l4 = encoder.layer1, encoder.layer2, encoder.layer3, encoder.layer4
        self.d4, self.d3 = Block(768, 256), Block(384, 128)
        self.d2, self.d1 = Block(192, 64), Block(128, 48)
        self.drop = nn.Dropout2d(0.1)
        self.head = nn.Conv2d(48, 3, 1)

    def forward(self, value: torch.Tensor) -> torch.Tensor:
        size = value.shape[-2:]
        e0 = self.stem(value)
        e1 = self.l1(self.pool(e0))
        e2, e3, e4 = self.l2(e1), None, None
        e3 = self.l3(e2)
        e4 = self.l4(e3)

        def up(source: torch.Tensor, reference: torch.Tensor) -> torch.Tensor:
            return functional.interpolate(source, size=reference.shape[-2:], mode="bilinear", align_corners=False)

        decoded = self.d4(torch.cat((up(e4, e3), e3), dim=1))
        decoded = self.d3(torch.cat((up(decoded, e2), e2), dim=1))
        decoded = self.d2(torch.cat((up(decoded, e1), e1), dim=1))
        decoded = self.d1(torch.cat((up(decoded, e0), e0), dim=1))
        return functional.interpolate(self.head(self.drop(decoded)), size=size, mode="bilinear", align_corners=False)


def sha256_bytes(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def source_validation_images(manifest: Path, count: int = 3) -> list[Path]:
    selected = []
    with manifest.open("r", encoding="utf-8-sig", newline="") as stream:
        for row in csv.DictReader(stream):
            if row["split"] == "validation":
                path = Path(row["image"])
                if not path.is_file():
                    raise FileNotFoundError(path)
                selected.append(path)
                if len(selected) == count:
                    break
    if len(selected) != count:
        raise ValueError("Unable to find three validation images for export parity")
    return selected


def normalized_input(rgb: np.ndarray, size: int) -> torch.Tensor:
    image = Image.fromarray(rgb, mode="RGB").convert("L").resize((size, size), Image.Resampling.BILINEAR)
    gray = np.asarray(image, dtype=np.float32) / 255.0
    value = np.repeat(gray[None, None, :, :], 3, axis=1)
    mean = np.array([0.485, 0.456, 0.406], dtype=np.float32)[None, :, None, None]
    std = np.array([0.229, 0.224, 0.225], dtype=np.float32)[None, :, None, None]
    return torch.from_numpy((value - mean) / std)


@torch.inference_mode()
def eager_predict(model: nn.Module, rgb: np.ndarray, metadata: dict) -> tuple[np.ndarray, np.ndarray]:
    size = int(metadata["eval_size"])
    tensor = normalized_input(rgb, size)
    bias = torch.tensor([metadata["logit_bias"][name] for name in CLASS_NAMES]).view(1, 3, 1, 1)
    probs = []
    for dims in (None, (-1,), (-2,)):
        sample = tensor if dims is None else torch.flip(tensor, dims)
        logits = model(sample).float()
        if dims is not None:
            logits = torch.flip(logits, dims)
        probs.append(logits.softmax(1))
    calibrated = (torch.stack(probs).mean(0).clamp_min(1e-12).log() + bias).softmax(1)
    calibrated = F.interpolate(calibrated, size=rgb.shape[:2], mode="bilinear", align_corners=False)
    confidence, labels = calibrated.max(1)
    return labels[0].byte().numpy(), confidence[0].numpy()


def benchmark(package: Path, sample: np.ndarray, provider: str, repeats: int) -> dict:
    process = psutil.Process()
    before_rss = process.memory_info().rss
    runtime = P502ThreeClassRuntime(package, provider=provider, max_input_age_s=30.0)
    after_load_rss = process.memory_info().rss
    if runtime.device.type == "cuda":
        torch.cuda.synchronize(runtime.device)
        torch.cuda.reset_peak_memory_stats(runtime.device)
    for _ in range(2):
        runtime.infer(sample, captured_monotonic_s=time.monotonic())
    times = []
    confidences = []
    for _ in range(repeats):
        result = runtime.infer(sample, captured_monotonic_s=time.monotonic())
        if result.stale or result.status not in ("INFERENCE_PASS", "LOW_CONFIDENCE_PIXELS"):
            raise RuntimeError(f"Unexpected benchmark inference status: {result.status}")
        if not set(np.unique(result.labels)).issubset({0, 1, 2, UNKNOWN_ID}):
            raise RuntimeError("Runtime produced an unsupported class ID")
        if not np.isfinite(result.confidence).all() or np.min(result.confidence) < 0 or np.max(result.confidence) > 1:
            raise RuntimeError("Runtime confidence is non-finite or outside [0,1]")
        times.append(result.latency_ms)
        confidences.append(float(np.mean(result.confidence)))
    gpu_memory = None
    if runtime.device.type == "cuda":
        torch.cuda.synchronize(runtime.device)
        gpu_memory = {
            "peak_allocated_mb": torch.cuda.max_memory_allocated(runtime.device) / (1024 ** 2),
            "peak_reserved_mb": torch.cuda.max_memory_reserved(runtime.device) / (1024 ** 2),
        }
    after_rss = process.memory_info().rss
    ordered = sorted(times)
    p95 = ordered[min(len(ordered) - 1, int(np.ceil(0.95 * len(ordered))) - 1)]
    return {
        "provider": str(runtime.device), "iterations": repeats,
        "warmup_iterations": 2,
        "provider_initialization_warmup_ms": runtime.warmup_latency_ms,
        "input_size": list(sample.shape[:2]),
        "latency_ms": {"p50": statistics.median(times), "p95": p95,
                       "min": min(times), "max": max(times)},
        "throughput_fps_at_p50": 1000.0 / statistics.median(times),
        "process_rss_mb": {"before_model_load": before_rss / (1024 ** 2),
                            "after_model_load": after_load_rss / (1024 ** 2),
                            "after_benchmark": after_rss / (1024 ** 2)},
        "gpu_memory_mb": gpu_memory,
        "mean_frame_confidence_avg": statistics.mean(confidences),
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--checkpoint", type=Path,
                        default=ROOT / ".build/p502-three-class-output-20261112/unet_r34_3class_best_ema.pt")
    parser.add_argument("--source-meta", type=Path,
                        default=ROOT / ".build/p502-three-class-output-20261112/model_meta.json")
    parser.add_argument("--manifest", type=Path,
                        default=ROOT / ".build/p502-three-class-dataset-20261112/manifest.csv")
    parser.add_argument("--package", type=Path,
                        default=ROOT / ".build/p502-threeclass-package-20261112")
    parser.add_argument("--evidence", type=Path,
                        default=ROOT / "evidence/runs/20261112-p503/P5-03")
    parser.add_argument("--repeats", type=int, default=8)
    args = parser.parse_args()
    if args.package.exists():
        raise FileExistsError(f"Refusing to overwrite package: {args.package}")
    if args.repeats < 5:
        raise ValueError("Use at least five benchmark repetitions")
    torch.set_num_threads(4)
    source_meta = json.loads(args.source_meta.read_text())
    if source_meta.get("class_names") != CLASS_NAMES or source_meta.get("eval_size") != 448:
        raise ValueError("Source checkpoint metadata does not match the P5-02 three-class contract")
    gate_path = ROOT / "config/perception/gates-three-class.json"
    gate = json.loads(gate_path.read_text())
    if source_meta["gate"]["gate_id"] != gate["gate_id"] or source_meta["gate"]["minimum_miou"] != gate["minimum_miou"] or source_meta["gate"]["minimum_per_class_iou"] != gate["minimum_per_class_iou"]:
        raise ValueError("Source export and frozen P5-02 gate do not match")

    package = args.package.resolve()
    package.mkdir(parents=True)
    model = UNetR34().eval()
    state = torch.load(args.checkpoint, map_location="cpu", weights_only=True)
    model.load_state_dict(state, strict=True)
    example = torch.zeros(1, 3, source_meta["eval_size"], source_meta["eval_size"])
    traced = torch.jit.trace(model, example, strict=True).eval()
    model_path = package / "model.ts"
    traced.save(str(model_path))
    model_sha = sha256_file(model_path)
    metadata = dict(source_meta)
    metadata.update({
        "schema_version": 1,
        "package_id": "p502-threeclass-torchscript-v1",
        "model_file": "model.ts",
        "model_sha256": model_sha,
        "source_checkpoint_sha256": sha256_file(args.checkpoint),
        "source_checkpoint": "all_outputs.zip:p502-threeclass-rock-recovery-v1/export/unet_r34_3class_best_ema.pt",
        "unknown_id": UNKNOWN_ID,
        "input_contract": "uint8 RGB HxWx3; original-resolution label/confidence output",
        "default_max_input_age_s": 1.0,
        "default_min_confidence": 0.0,
        "runtime": {"framework": "PyTorch TorchScript", "torch_version": torch.__version__,
                    "providers": ["cpu"] + (["cuda"] if torch.cuda.is_available() else [])},
    })
    (package / "model_meta.json").write_text(json.dumps(metadata, indent=2) + "\n")
    shutil.copy2(ROOT / "scripts/p502_threeclass_inference.py", package / "p502_threeclass_inference.py")

    # Verify raw-logit parity on deterministic inputs before packaging.
    parity = []
    with torch.inference_mode():
        for name, tensor in (("zeros", torch.zeros_like(example)),
                             ("midgray", torch.full_like(example, 0.25)),
                             ("ramp", torch.linspace(-2.0, 2.0, example.numel()).reshape_as(example))):
            ref = model(tensor)
            got = traced(tensor)
            difference = (ref - got).abs()
            entry = {"input": name, "max_abs_logit_error": float(difference.max()),
                     "mean_abs_logit_error": float(difference.mean()),
                     "allclose_atol_1e-5_rtol_1e-4": bool(torch.allclose(ref, got, atol=1e-5, rtol=1e-4))}
            if not entry["allclose_atol_1e-5_rtol_1e-4"]:
                raise RuntimeError(f"TorchScript export parity failed on {name}: {entry}")
            parity.append(entry)

    # End-to-end eager/script parity on held-out-free validation examples.
    runtime = P502ThreeClassRuntime(package, provider="cpu", max_input_age_s=30.0)
    pipeline_parity = []
    for path in source_validation_images(args.manifest):
        with Image.open(path) as image:
            rgb = np.asarray(image.convert("RGB"), dtype=np.uint8)
        eager_labels, eager_conf = eager_predict(model, rgb, metadata)
        exported = runtime.infer(rgb, captured_monotonic_s=time.monotonic())
        disagreement = float(np.mean(eager_labels != exported.labels))
        confidence_error = float(np.max(np.abs(eager_conf - exported.confidence)))
        entry = {"image_sha256": sha256_file(path), "label_disagreement_fraction": disagreement,
                 "max_abs_confidence_error": confidence_error}
        if disagreement > 0.0001 or confidence_error > 0.0002:
            raise RuntimeError(f"End-to-end export parity failed: {entry}")
        pipeline_parity.append(entry)

    # A stale frame must be rejected before inference and rendered unknown.
    stale_runtime = P502ThreeClassRuntime(package, provider="cpu", max_input_age_s=0.5)
    sample_path = source_validation_images(args.manifest, 1)[0]
    with Image.open(sample_path) as image:
        sample = np.asarray(image.convert("RGB"), dtype=np.uint8)
    stale = stale_runtime.infer(sample, captured_monotonic_s=time.monotonic() - 1.0)
    if not stale.stale or stale.status != "STALE_INPUT" or not np.all(stale.labels == UNKNOWN_ID):
        raise RuntimeError("Stale-frame safety contract failed")

    providers = ["cpu"] + (["cuda"] if torch.cuda.is_available() else [])
    benchmark_results = [benchmark(package, sample, provider, args.repeats) for provider in providers]
    readme = f"""# P5-02 three-class inference package

Model: ResNet34 U-Net TorchScript export. Output class IDs: 0 regolith, 1 rock,
2 shadow; 255 unknown. Bedrock labels were merged into rock during training;
crater and ignored source pixels were excluded. Crater risk remains the
geometry layer's responsibility.

Input: uint8 RGB HxWx3. Runtime preprocessing follows the qualified checkpoint:
grayscale conversion, bilinear {metadata['eval_size']}x{metadata['eval_size']} resize,
ImageNet normalization, and identity/horizontal/vertical flip probability
averaging. Output labels and maximum-class confidence retain input resolution.

Load `p502_threeclass_inference.py` and construct
`P502ThreeClassRuntime(package_path, provider='auto')`. Supply the image capture
time from the same monotonic clock using `captured_monotonic_s`. Frames older
than `max_input_age_s` return all-255 labels and zero confidence with
`STALE_INPUT`. `min_confidence` is configurable; the default 0.0 exposes raw
confidence without discarding any predicted pixels. Provider, inference time,
input age, stale state, and status are returned for each frame. Provider
warmup runs during construction so first-frame initialization is not charged
to the input freshness budget.

The model SHA-256 is `{model_sha}`. Package metadata binds the TorchScript file
to this hash and records the exact P5-02 source checkpoint hash.
"""
    (package / "README.md").write_text(readme)

    archive_path = args.evidence / "p502-threeclass-runtime.zip"
    if archive_path.exists():
        raise FileExistsError(f"Refusing to overwrite package archive: {archive_path}")
    args.evidence.mkdir(parents=True, exist_ok=True)
    with zipfile.ZipFile(archive_path, "w", compression=zipfile.ZIP_DEFLATED,
                         compresslevel=1, allowZip64=True) as archive:
        for path in sorted(package.iterdir()):
            archive.write(path, path.name)
    result = {
        "task_id": "P5-03", "result": "PASS", "package_id": metadata["package_id"],
        "source_checkpoint_sha256": metadata["source_checkpoint_sha256"],
        "torchscript_sha256": model_sha, "package_archive": str(archive_path.relative_to(ROOT)),
        "package_archive_sha256": sha256_file(archive_path),
        "package_archive_bytes": archive_path.stat().st_size,
        "source_metadata_sha256": sha256_file(args.source_meta),
        "gate_id": gate["gate_id"], "classes": CLASS_NAMES,
        "export_parity": {"raw_logit_tests": parity, "e2e_validation_frames": pipeline_parity,
                          "result": "PASS"},
        "confidence": {"range": [0.0, 1.0], "type": "maximum calibrated class probability",
                       "confidence_threshold_default": 0.0,
                       "parity_max_abs_error": max(x["max_abs_confidence_error"] for x in pipeline_parity)},
        "staleness": {"default_max_input_age_s": 1.0,
                      "verification": {"status": stale.status, "stale": stale.stale,
                                       "all_labels_unknown": bool(np.all(stale.labels == UNKNOWN_ID)),
                                       "max_age_s": stale_runtime.max_input_age_s}},
        "providers": benchmark_results,
        "runtime_package_files": [path.name for path in sorted(package.iterdir())],
        "limitations": ["Latency and memory are local single-stream measurements; ROS end-to-end latency remains to be measured.",
                        "Confidence is exposed for downstream policy; no calibrated reject threshold is claimed.",
                        "P5-02 analog-corpus scope and limitations remain unchanged."],
    }
    (args.evidence / "verification.json").write_text(json.dumps(result, indent=2) + "\n")
    (args.evidence / "report.md").write_text(
        "# P5-03 TorchScript export and inference verification\n\n"
        f"Result: **PASS**. Package `{metadata['package_id']}` preserves the P5-02 three-class contract.\n\n"
        f"Checkpoint SHA-256: `{metadata['source_checkpoint_sha256']}`  \n"
        f"TorchScript SHA-256: `{model_sha}`  \n"
        f"Package archive: `{archive_path.relative_to(ROOT)}`  \n"
        f"Package SHA-256: `{result['package_archive_sha256']}`\n\n"
        "Raw logits passed deterministic eager-versus-TorchScript parity. End-to-end labels and confidence "
        "passed on three validation images. Runtime exposes CPU/CUDA provider, latency, confidence and input age. "
        "A deliberately stale frame returned status `STALE_INPUT` and only unknown labels.\n\n"
        "CPU/CUDA latency and memory measurements are in `verification.json`. These are single-stream local "
        "measurements; ROS end-to-end latency is not included. Confidence is reported but no calibrated reject "
        "threshold is claimed.\n"
    )
    print(json.dumps({"result": "PASS", "package": str(package),
                      "archive": str(archive_path), "torchscript_sha256": model_sha,
                      "providers": [{"provider": p["provider"], "p50_ms": p["latency_ms"]["p50"],
                                     "p95_ms": p["latency_ms"]["p95"],
                                     "rss_mb": p["process_rss_mb"]["after_benchmark"],
                                     "gpu_memory_mb": p["gpu_memory_mb"]} for p in benchmark_results]}, indent=2), flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
