"""Run the imported P5-02 five-class checkpoint on RGB camera frames."""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np
import torch
from PIL import Image
from torch import nn
from torch.nn import functional as F
import torchvision


class _Block(nn.Module):
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


class _UNetR34(nn.Module):
    def __init__(self) -> None:
        super().__init__()
        encoder = torchvision.models.resnet34(weights=None)
        self.stem = nn.Sequential(encoder.conv1, encoder.bn1, encoder.relu)
        self.pool = encoder.maxpool
        self.l1, self.l2 = encoder.layer1, encoder.layer2
        self.l3, self.l4 = encoder.layer3, encoder.layer4
        self.d4, self.d3 = _Block(768, 256), _Block(384, 128)
        self.d2, self.d1 = _Block(192, 64), _Block(128, 48)
        self.drop = nn.Dropout2d(0.1)
        self.head = nn.Conv2d(48, 5, 1)

    def forward(self, value: torch.Tensor) -> torch.Tensor:
        size = value.shape[-2:]
        e0 = self.stem(value)
        e1 = self.l1(self.pool(e0))
        e2, e3, e4 = self.l2(e1), None, None
        e3 = self.l3(e2)
        e4 = self.l4(e3)

        def up(source: torch.Tensor, reference: torch.Tensor) -> torch.Tensor:
            return F.interpolate(source, size=reference.shape[-2:], mode="bilinear", align_corners=False)

        decoded = self.d4(torch.cat((up(e4, e3), e3), dim=1))
        decoded = self.d3(torch.cat((up(decoded, e2), e2), dim=1))
        decoded = self.d2(torch.cat((up(decoded, e1), e1), dim=1))
        decoded = self.d1(torch.cat((up(decoded, e0), e0), dim=1))
        return F.interpolate(self.head(self.drop(decoded)), size=size, mode="bilinear", align_corners=False)


class P502SemanticModel:
    """Five-class prediction with the export's preprocessing and calibration."""

    def __init__(self, checkpoint: Path, device: str = "cpu") -> None:
        self.device = torch.device(device)
        self.metadata = json.loads(checkpoint.with_name("model_meta.json").read_text())
        if self.metadata["class_names"] != ["regolith", "bedrock", "rock", "crater", "shadow"]:
            raise ValueError("P5-02 export class order is unsupported")
        self.size = int(self.metadata["eval_size"])
        self.bias = torch.tensor(
            [self.metadata["logit_bias"][name] for name in self.metadata["class_names"]],
            device=self.device,
        ).view(1, 5, 1, 1)
        state = torch.load(checkpoint, map_location="cpu", weights_only=True)
        self.model = _UNetR34().to(self.device).eval()
        self.model.load_state_dict(state)

    @torch.inference_mode()
    def predict(self, rgb: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        image = Image.fromarray(rgb).convert("L").resize(
            (self.size, self.size), Image.Resampling.BILINEAR)
        gray = np.asarray(image, dtype=np.float32) / 255.0
        value = np.repeat(gray[None, None, :, :], 3, axis=1)
        mean = np.array([0.485, 0.456, 0.406], dtype=np.float32)[None, :, None, None]
        std = np.array([0.229, 0.224, 0.225], dtype=np.float32)[None, :, None, None]
        tensor = torch.from_numpy((value - mean) / std).to(self.device)
        probabilities = []
        for flip_dims in (None, (-1,), (-2,)):
            sample = tensor if flip_dims is None else torch.flip(tensor, flip_dims)
            logits = self.model(sample)
            if flip_dims is not None:
                logits = torch.flip(logits, flip_dims)
            probabilities.append(logits.softmax(dim=1))
        averaged = torch.stack(probabilities).mean(dim=0).clamp_min(1e-12)
        calibrated = (averaged.log() + self.bias).softmax(dim=1)
        calibrated = F.interpolate(calibrated, size=rgb.shape[:2], mode="bilinear", align_corners=False)
        confidence, labels = calibrated.max(dim=1)
        return labels[0].byte().cpu().numpy(), confidence[0].cpu().numpy()
