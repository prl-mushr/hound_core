"""Shared helpers for the hound_core segmentation nodes."""

import cv2
import numpy as np
import torch

# Category colors: label i (1-indexed) -> PALETTE[(i-1) % len]; label 0 = none.
# Default order matches traversable -> green, non_traversable -> red, people -> blue.
PALETTE = [(0, 200, 0), (220, 0, 0), (0, 80, 255),
           (230, 230, 0), (0, 230, 230), (230, 0, 230)]


def extract_boxes(binary_np: np.ndarray, min_area: float, max_boxes: int):
    """Per-class binary masks [n,H,W] (uint8) -> the largest blobs as a list of
    (class_idx, area, (x0, y0, x1, y1)), biggest first, capped at max_boxes."""
    cands = []
    for i in range(binary_np.shape[0]):
        ncomp, _, stats, _ = cv2.connectedComponentsWithStats(binary_np[i], connectivity=8)
        for c in range(1, ncomp):  # skip background label 0
            area = int(stats[c, cv2.CC_STAT_AREA])
            if area < min_area:
                continue
            x = int(stats[c, cv2.CC_STAT_LEFT])
            y = int(stats[c, cv2.CC_STAT_TOP])
            bw = int(stats[c, cv2.CC_STAT_WIDTH])
            bh = int(stats[c, cv2.CC_STAT_HEIGHT])
            cands.append((i, area, (x, y, x + bw, y + bh)))
    cands.sort(key=lambda t: t[1], reverse=True)
    return cands[:max_boxes]


def labels_to_boxes(labels: np.ndarray, n_groups: int, min_area_frac: float, max_boxes: int):
    """Label map [H,W] (0=none, i=group i-1) -> box prompts for SAM refinement."""
    h, w = labels.shape
    binary = np.zeros((n_groups, h, w), dtype=np.uint8)
    for i in range(n_groups):
        binary[i] = (labels == (i + 1)).astype(np.uint8)
    return extract_boxes(binary, min_area_frac * h * w, max_boxes)


def blend_overlay(base_rgb: np.ndarray, color_rgb: np.ndarray,
                  conf: np.ndarray, alpha: float) -> np.ndarray:
    """Blend color_rgb over base_rgb, weighted per-pixel by conf*alpha so flat /
    low-confidence regions stay close to the original. All HxW(x3); -> HxWx3 uint8."""
    strength = (conf * alpha)[:, :, None]
    out = base_rgb.astype(np.float32) * (1.0 - strength) + color_rgb.astype(np.float32) * strength
    return np.clip(out, 0, 255).astype(np.uint8)


def to_image(bridge, array: np.ndarray, encoding: str, header):
    """cv2_to_imgmsg + stamp with the given header, in one call."""
    msg = bridge.cv2_to_imgmsg(array, encoding=encoding)
    msg.header = header
    return msg


def stamp_key(sec: int, nanosec: int) -> int:
    """Pack a ROS time into a single int key (for matching frames by stamp)."""
    return int(sec) * 1_000_000_000 + int(nanosec)


def probs_to_labels(probs: torch.Tensor, threshold: float):
    """Per-group prob maps [n,H,W] -> a single label map. Each pixel takes the
    argmax group (label = group_index + 1), or 0 (none) where the best group is
    below threshold. Returns (labels uint8 [H,W], confidence float [H,W]) numpy."""
    maxv, arg = probs.max(dim=0)
    labels = (arg + 1).to(torch.uint8)
    labels = torch.where(maxv >= threshold, labels, torch.zeros_like(labels))
    return labels.cpu().numpy(), maxv.float().cpu().numpy()


def labels_to_color(labels: np.ndarray, n_groups: int) -> np.ndarray:
    """Color a label map [H,W] via PALETTE (label 0 stays black). -> HxWx3 uint8."""
    out = np.zeros((*labels.shape, 3), dtype=np.uint8)
    for i in range(n_groups):
        out[labels == (i + 1)] = PALETTE[i % len(PALETTE)]
    return out
