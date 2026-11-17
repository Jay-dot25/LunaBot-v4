#!/usr/bin/env python3
"""Dependency-light confusion matrix and IoU metrics for indexed segmentation masks."""
from __future__ import annotations
from typing import Iterable
import numpy as np


def confusion_matrix(target: np.ndarray, prediction: np.ndarray, class_count: int = 5, ignore_id: int = 255) -> np.ndarray:
    """Return rows=true, columns=predicted; ignore target pixels marked 255."""
    truth = np.asarray(target).reshape(-1).astype(np.int64, copy=False)
    pred = np.asarray(prediction).reshape(-1).astype(np.int64, copy=False)
    if truth.shape != pred.shape:
        raise ValueError("target and prediction must have the same number of pixels")
    valid = truth != ignore_id
    truth, pred = truth[valid], pred[valid]
    if np.any((truth < 0) | (truth >= class_count)):
        raise ValueError("target contains an unsupported class ID")
    if np.any((pred < 0) | (pred >= class_count)):
        raise ValueError("prediction contains an unsupported class ID")
    encoded = truth * class_count + pred
    return np.bincount(encoded, minlength=class_count * class_count).reshape(class_count, class_count)


def summarize_confusion(matrix: np.ndarray, class_names: Iterable[str]) -> dict[str, object]:
    """Compute per-class support/IoU and mIoU over supported classes only."""
    matrix = np.asarray(matrix, dtype=np.int64)
    names = list(class_names)
    if matrix.shape != (len(names), len(names)):
        raise ValueError("confusion matrix shape must match class_names")
    if np.any(matrix < 0):
        raise ValueError("confusion matrix cannot contain negative counts")
    tp = np.diag(matrix)
    support = matrix.sum(axis=1)
    predicted = matrix.sum(axis=0)
    union = support + predicted - tp
    ious = [float(tp[i] / union[i]) if union[i] else None for i in range(len(names))]
    valid_ious = [value for value in ious if value is not None]
    return {
        "class_names": names,
        "confusion_matrix": matrix.tolist(),
        "confusion_orientation": "rows=true, columns=predicted; ignore target pixels excluded",
        "per_class": {
            name: {"iou": ious[i], "support_pixels": int(support[i]), "predicted_pixels": int(predicted[i]), "union_pixels": int(union[i])}
            for i, name in enumerate(names)
        },
        "miou": float(sum(valid_ious) / len(valid_ious)) if valid_ious else None,
        "miou_supported_class_count": len(valid_ious),
    }


def bootstrap_scene_iou(matrices, class_names, draws=2000, seed=20261112):
    """Resample complete scene confusion matrices with replacement."""
    names = list(class_names)
    scenes = np.asarray(matrices, dtype=np.int64)
    if scenes.ndim != 3 or scenes.shape[1:] != (len(names), len(names)) or (scenes < 0).any():
        raise ValueError('Expected one nonnegative confusion matrix per scene')
    supporting_scenes = (scenes.sum(axis=2) > 0).sum(axis=0)
    rng = np.random.default_rng(seed)
    samples = np.full((draws, len(names)), np.nan)
    for i in range(draws):
        pooled = scenes[rng.integers(len(scenes), size=len(scenes))].sum(axis=0)
        tp = pooled.diagonal()
        support = pooled.sum(axis=1)
        union = support + pooled.sum(axis=0) - tp
        np.divide(tp, union, out=samples[i], where=support > 0)

    def interval(values, support_count):
        valid = values[np.isfinite(values)]
        bounds = np.quantile(valid, [.025, .975]).tolist() if support_count >= 3 and len(valid) >= .95 * draws else None
        return dict(ci95=bounds, positive_scenes=int(support_count), valid_draws=len(valid))

    return dict(scene_count=len(scenes), draws=draws, seed=seed,
                per_class={name: interval(samples[:, i], supporting_scenes[i]) for i, name in enumerate(names)},
                miou=interval(samples.mean(axis=1), supporting_scenes.min()))
