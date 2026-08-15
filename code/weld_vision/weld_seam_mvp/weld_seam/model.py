from __future__ import annotations

import torch
from torch import nn
import torch.nn.functional as functional


def _group_count(channels: int) -> int:
    for groups in (8, 4, 2):
        if channels % groups == 0:
            return groups
    return 1


class ConvBlock(nn.Module):
    def __init__(self, in_channels: int, out_channels: int) -> None:
        super().__init__()
        groups = _group_count(out_channels)
        self.layers = nn.Sequential(
            nn.Conv2d(in_channels, out_channels, 3, padding=1, bias=False),
            nn.GroupNorm(groups, out_channels),
            nn.SiLU(inplace=True),
            nn.Conv2d(out_channels, out_channels, 3, padding=1, bias=False),
            nn.GroupNorm(groups, out_channels),
            nn.SiLU(inplace=True),
        )

    def forward(self, inputs: torch.Tensor) -> torch.Tensor:
        return self.layers(inputs)


class DownBlock(nn.Module):
    def __init__(self, in_channels: int, out_channels: int) -> None:
        super().__init__()
        self.layers = nn.Sequential(nn.MaxPool2d(2), ConvBlock(in_channels, out_channels))

    def forward(self, inputs: torch.Tensor) -> torch.Tensor:
        return self.layers(inputs)


class UpBlock(nn.Module):
    def __init__(self, in_channels: int, skip_channels: int, out_channels: int) -> None:
        super().__init__()
        self.conv = ConvBlock(in_channels + skip_channels, out_channels)

    def forward(self, inputs: torch.Tensor, skip: torch.Tensor) -> torch.Tensor:
        inputs = functional.interpolate(
            inputs, size=skip.shape[-2:], mode="bilinear", align_corners=False
        )
        return self.conv(torch.cat((skip, inputs), dim=1))


class TinyUNet(nn.Module):
    """A compact U-Net that is practical for CPU training and edge deployment."""

    def __init__(self, in_channels: int = 3, base_channels: int = 16) -> None:
        super().__init__()
        b = base_channels
        self.input_block = ConvBlock(in_channels, b)
        self.down1 = DownBlock(b, b * 2)
        self.down2 = DownBlock(b * 2, b * 4)
        self.down3 = DownBlock(b * 4, b * 8)
        self.down4 = DownBlock(b * 8, b * 12)
        self.up1 = UpBlock(b * 12, b * 8, b * 8)
        self.up2 = UpBlock(b * 8, b * 4, b * 4)
        self.up3 = UpBlock(b * 4, b * 2, b * 2)
        self.up4 = UpBlock(b * 2, b, b)
        self.output = nn.Conv2d(b, 1, kernel_size=1)

    def forward(self, inputs: torch.Tensor) -> torch.Tensor:
        x0 = self.input_block(inputs)
        x1 = self.down1(x0)
        x2 = self.down2(x1)
        x3 = self.down3(x2)
        x4 = self.down4(x3)
        outputs = self.up1(x4, x3)
        outputs = self.up2(outputs, x2)
        outputs = self.up3(outputs, x1)
        outputs = self.up4(outputs, x0)
        return self.output(outputs)


def build_model(base_channels: int = 16) -> TinyUNet:
    return TinyUNet(base_channels=base_channels)
