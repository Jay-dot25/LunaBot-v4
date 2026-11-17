"""Runtime API for the qualified P5-02 regolith/rock/shadow model package."""
from __future__ import annotations

from dataclasses import dataclass
import hashlib
import json
from pathlib import Path
import time

import numpy as np
import torch
import torch.nn.functional as F
from PIL import Image


CLASS_NAMES = ["regolith", "rock", "shadow"]
UNKNOWN_ID = 255


@dataclass(frozen=True)
class InferenceResult:
    labels: np.ndarray
    confidence: np.ndarray
    provider: str
    latency_ms: float
    input_age_ms: float
    stale: bool
    status: str


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


class P502ThreeClassRuntime:
    """TorchScript semantic inference with explicit freshness and confidence."""

    def __init__(self, package: Path, provider: str = "auto", *,
                 max_input_age_s: float = 1.0, min_confidence: float = 0.0) -> None:
        init_started = time.perf_counter()
        self.package = Path(package)
        self.metadata = json.loads((self.package / "model_meta.json").read_text())
        if self.metadata.get("class_names") != CLASS_NAMES:
            raise ValueError("P5-03 model class order is unsupported")
        if self.metadata.get("unknown_id") != UNKNOWN_ID:
            raise ValueError("P5-03 unknown label must be 255")
        if not 0.0 <= min_confidence <= 1.0:
            raise ValueError("min_confidence must be between zero and one")
        if max_input_age_s <= 0:
            raise ValueError("max_input_age_s must be positive")
        self.max_input_age_s = float(max_input_age_s)
        self.min_confidence = float(min_confidence)

        if provider == "auto":
            provider = "cuda" if torch.cuda.is_available() else "cpu"
        if provider not in ("cpu", "cuda"):
            raise ValueError("provider must be auto, cpu, or cuda")
        if provider == "cuda" and not torch.cuda.is_available():
            raise RuntimeError("CUDA provider was requested but is unavailable")
        self.device = torch.device(provider)
        model_path = self.package / self.metadata["model_file"]
        if sha256_file(model_path) != self.metadata["model_sha256"]:
            raise ValueError("TorchScript model hash does not match package metadata")
        self.model = torch.jit.load(str(model_path), map_location=self.device).eval()
        self.bias = torch.tensor(
            [self.metadata["logit_bias"][name] for name in CLASS_NAMES],
            dtype=torch.float32, device=self.device).view(1, 3, 1, 1)
        self.input_size = int(self.metadata["eval_size"])
        self.warmup_latency_ms = self._warmup()

    @torch.inference_mode()
    def _warmup(self) -> float:
        """Initialize the selected provider before processing fresh camera frames."""
        started = time.perf_counter()
        sample = torch.zeros((1, 3, self.input_size, self.input_size),
                             dtype=torch.float32, device=self.device)
        for dims in (None, (-1,), (-2,)):
            view = sample if dims is None else torch.flip(sample, dims)
            self.model(view).float().softmax(dim=1)
        if self.device.type == "cuda":
            torch.cuda.synchronize(self.device)
        return (time.perf_counter() - started) * 1000.0

    def _unknown(self, shape: tuple[int, int], provider: str,
                 latency_ms: float, age_ms: float, stale: bool,
                 status: str) -> InferenceResult:
        return InferenceResult(
            np.full(shape, UNKNOWN_ID, dtype=np.uint8),
            np.zeros(shape, dtype=np.float32), provider, latency_ms, age_ms,
            stale, status)

    @torch.inference_mode()
    def infer(self, rgb: np.ndarray, *, captured_monotonic_s: float | None = None) -> InferenceResult:
        """Infer one uint8 RGB frame; stale frames yield all-unknown outputs.

        `captured_monotonic_s` must come from the same monotonic clock as the
        caller. Omitting it marks the frame as captured at this call's start.
        """
        started = time.perf_counter()
        if not isinstance(rgb, np.ndarray) or rgb.dtype != np.uint8 or rgb.ndim != 3 or rgb.shape[2] != 3:
            raise ValueError("rgb must be an HxWx3 uint8 RGB array")
        height, width = rgb.shape[:2]
        if height < 1 or width < 1:
            raise ValueError("rgb frame must not be empty")
        now = time.monotonic()
        captured = now if captured_monotonic_s is None else float(captured_monotonic_s)
        age_s = now - captured
        provider = str(self.device)
        if age_s < -0.05:
            return self._unknown((height, width), provider,
                                 (time.perf_counter() - started) * 1000.0,
                                 age_s * 1000.0, False, "INVALID_FUTURE_TIMESTAMP")
        if age_s > self.max_input_age_s:
            return self._unknown((height, width), provider,
                                 (time.perf_counter() - started) * 1000.0,
                                 age_s * 1000.0, True, "STALE_INPUT")

        image = Image.fromarray(rgb, mode="RGB").convert("L").resize(
            (self.input_size, self.input_size), Image.Resampling.BILINEAR)
        gray = np.asarray(image, dtype=np.float32) / 255.0
        value = np.repeat(gray[None, None, :, :], 3, axis=1)
        mean = np.array([0.485, 0.456, 0.406], dtype=np.float32)[None, :, None, None]
        std = np.array([0.229, 0.224, 0.225], dtype=np.float32)[None, :, None, None]
        tensor = torch.from_numpy((value - mean) / std).to(self.device)

        probabilities = []
        for dims in (None, (-1,), (-2,)):
            sample = tensor if dims is None else torch.flip(tensor, dims)
            logits = self.model(sample).float()
            if dims is not None:
                logits = torch.flip(logits, dims)
            probabilities.append(logits.softmax(dim=1))
        averaged = torch.stack(probabilities).mean(dim=0).clamp_min(1e-12)
        calibrated = (averaged.log() + self.bias).softmax(dim=1)
        calibrated = F.interpolate(calibrated, size=(height, width), mode="bilinear", align_corners=False)
        confidence, labels = calibrated.max(dim=1)
        labels_np = labels[0].byte().cpu().numpy()
        confidence_np = confidence[0].cpu().numpy().astype(np.float32, copy=False)
        uncertain = confidence_np < self.min_confidence
        labels_np[uncertain] = UNKNOWN_ID
        status = "LOW_CONFIDENCE_PIXELS" if np.any(uncertain) else "INFERENCE_PASS"
        if self.device.type == "cuda":
            torch.cuda.synchronize(self.device)
        latency_ms = (time.perf_counter() - started) * 1000.0
        age_ms = (time.monotonic() - captured) * 1000.0
        stale = age_ms > self.max_input_age_s * 1000.0
        if stale:
            return self._unknown((height, width), provider, latency_ms, age_ms, True, "STALE_DURING_INFERENCE")
        return InferenceResult(labels_np, confidence_np, provider, latency_ms,
                               age_ms, False, status)
