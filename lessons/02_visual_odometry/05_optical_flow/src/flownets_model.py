"""Inference for the PTLFlow FlowNetS 'things' checkpoint."""
import cv2
import numpy as np
import torch
from torch import nn
from torch.nn import functional as F

WEIGHTS = ("https://github.com/hmorimitsu/ptlflow/releases/download/weights1/"
           "flownets-things-98cde14d.ckpt")


def encoder(in_channels, out_channels, kernel=3, stride=1):
    return nn.Sequential(
        nn.Conv2d(in_channels, out_channels, kernel, stride, kernel // 2),
        nn.LeakyReLU(0.1),
    )


class FlowNetS(nn.Module):
    """Six encoder levels followed by coarse-to-fine flow prediction."""
    def __init__(self):
        super().__init__()
        self.conv1 = encoder(6, 64, 7, 2)
        self.conv2 = encoder(64, 128, 5, 2)
        self.conv3 = encoder(128, 256, 5, 2)
        self.conv3_1 = encoder(256, 256)
        self.conv4 = encoder(256, 512, stride=2)
        self.conv4_1 = encoder(512, 512)
        self.conv5 = encoder(512, 512, stride=2)
        self.conv5_1 = encoder(512, 512)
        self.conv6 = encoder(512, 1024, stride=2)
        self.conv6_1 = encoder(1024, 1024)

        # Names and channel counts match the released checkpoint.
        self.predict_flow6 = nn.Conv2d(1024, 2, 3, padding=1)
        for level, inputs, features, skip in (
            (5, 1024, 512, 512), (4, 1026, 256, 512),
            (3, 770, 128, 256), (2, 386, 64, 128),
        ):
            self.add_module(f"deconv{level}", nn.Sequential(
                nn.ConvTranspose2d(inputs, features, 4, stride=2, padding=1), nn.LeakyReLU(0.1)))
            self.add_module(f"upsampled_flow{level + 1}_to_{level}",
                            nn.ConvTranspose2d(2, 2, 4, stride=2, padding=1, bias=False))
            self.add_module(f"predict_flow{level}", nn.Conv2d(skip + features + 2, 2, 3, padding=1))

    def forward(self, pair):
        c2 = self.conv2(self.conv1(pair))
        c3 = self.conv3_1(self.conv3(c2))
        c4 = self.conv4_1(self.conv4(c3))
        c5 = self.conv5_1(self.conv5(c4))
        features = self.conv6_1(self.conv6(c5))
        flow = self.predict_flow6(features)
        for level, skip in zip((5, 4, 3, 2), (c5, c4, c3, c2)):
            up_features = getattr(self, f"deconv{level}")(features)
            up_flow = getattr(self, f"upsampled_flow{level + 1}_to_{level}")(flow)
            features = torch.cat((skip, up_features, up_flow), dim=1)
            flow = getattr(self, f"predict_flow{level}")(features)
        return 20.0 * F.interpolate(flow, scale_factor=4, mode="bilinear", align_corners=False)


def load_model(cache_dir, device):
    checkpoint = torch.hub.load_state_dict_from_url(
        WEIGHTS, model_dir=str(cache_dir), map_location="cpu", check_hash=True, weights_only=True)
    model = FlowNetS()
    model.load_state_dict(checkpoint["state_dict"])
    return model.eval().to(device)


@torch.inference_mode()
def estimate_flow(model, previous, current, device):
    """Return forward (dx, dy) in the original image's pixel coordinates."""
    height, width = previous.shape[:2]
    rgb = np.stack([cv2.cvtColor(frame, cv2.COLOR_BGR2RGB) for frame in (previous, current)])
    images = torch.from_numpy(rgb).to(device=device, dtype=torch.float32).permute(0, 3, 1, 2) / 255
    images = images - images.mean(dim=(2, 3), keepdim=True)
    net_h, net_w = ((size + 63) // 64 * 64 for size in (height, width))
    images = F.interpolate(images, size=(net_h, net_w), mode="bilinear", align_corners=True)
    field = model(images.reshape(1, 6, net_h, net_w))
    field = F.interpolate(field, size=(height, width), mode="bilinear", align_corners=True)
    field = field[0].permute(1, 2, 0).cpu().numpy()
    field *= np.array([width / net_w, height / net_h], dtype=np.float32)
    if not np.isfinite(field).all():
        raise ValueError("FlowNetS produced non-finite flow.")
    return field
