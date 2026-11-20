"""Small supervised cost-volume stereo network. RGB in [0,1], left disparity in pixels."""

import torch
from torch import nn
from torch.nn import functional as F


class CompactStereo(nn.Module):
    def __init__(self, max_disparity=128, channels=12):
        super().__init__()
        if max_disparity % 4 or max_disparity < 16:
            raise ValueError("Maximum disparity must be a multiple of four")
        self.max_disparity = max_disparity
        self.features = nn.Sequential(
            nn.Conv2d(3, channels, 3, stride=2, padding=1),
            nn.ReLU(),
            nn.Conv2d(channels, channels, 3, stride=2, padding=1),
            nn.ReLU(),
            nn.Conv2d(channels, channels, 3, padding=1),
            nn.ReLU(),
        )
        self.regularizer = nn.Sequential(
            nn.Conv3d(channels + 3, 8, 3, padding=1),
            nn.ReLU(),
            nn.Conv3d(8, 8, 3, padding=1),
            nn.ReLU(),
            nn.Conv3d(8, 1, 3, padding=1),
        )
        self.refine = nn.Sequential(
            nn.Conv2d(4, 16, 3, padding=1),
            nn.ReLU(),
            nn.Conv2d(16, 16, 3, padding=1),
            nn.ReLU(),
            nn.Conv2d(16, 1, 3, padding=1),
        )

    def forward(self, left, right):
        a, b = self.features(left), self.features(right)
        lcolor = F.avg_pool2d(left, 4)
        rcolor = F.avg_pool2d(right, 4)
        volumes, penalties = [], []
        width = a.shape[-1]
        columns = torch.arange(width, device=left.device).view(1, 1, 1, width)
        for offset in range(self.max_disparity // 4):
            shifted = F.pad(b, (offset, 0))[..., :width]
            shifted_color = F.pad(rcolor, (offset, 0))[..., :width]
            volumes.append(torch.cat(((a - shifted).abs(), (lcolor - shifted_color).abs()), 1))
            penalties.append((columns < offset).to(left.dtype) * 10000)
        costs = torch.stack(volumes, 2)
        scores = -self.regularizer(costs).squeeze(1)
        scores = scores - torch.cat(penalties, 1)
        probability = torch.softmax(scores, 1)
        values = torch.arange(self.max_disparity // 4, device=left.device).view(1, -1, 1, 1)
        coarse = (probability * values).sum(1, keepdim=True) * 4
        disparity = F.interpolate(
            coarse, size=left.shape[-2:], mode="bilinear", align_corners=False
        )
        residual = self.refine(torch.cat((left, disparity / self.max_disparity), 1))
        disparity = (disparity + 4 * torch.tanh(residual)).clamp(0, self.max_disparity - 1)
        confidence = F.interpolate(
            probability.amax(1, keepdim=True),
            size=left.shape[-2:],
            mode="bilinear",
            align_corners=False,
        )
        return disparity, confidence
