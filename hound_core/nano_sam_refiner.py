"""NanoSAM refinement helper.

Drop-in replacement for SamRefiner:
  refine(rgb_np, classes, boxes_xyxy, n_classes) -> [n_classes,H,W] torch float mask

Uses NanoSAM TensorRT engines (see colcon_ws/src/nanosam/data/).
Requires the nanosam Python package (colcon_ws/src/nanosam-src) and torch2trt.
"""

import numpy as np
import PIL.Image
import torch


class NanoSamRefiner:
    def __init__(
        self,
        image_encoder: str,
        mask_decoder: str,
        device: str = "cuda",
        logger=None,
    ):
        self.dev = device
        self.image_encoder = image_encoder
        self.mask_decoder = mask_decoder
        self.log = logger
        self.predictor = self._build()
        self.warmup()

    def _info(self, msg):
        if self.log:
            self.log.info(msg)

    def _warn(self, msg):
        if self.log:
            self.log.warn(msg)

    def _build(self):
        from nanosam.utils.predictor import Predictor

        self._info(f"NanoSAM image_encoder={self.image_encoder}")
        self._info(f"NanoSAM mask_decoder={self.mask_decoder}")

        return Predictor(self.image_encoder, self.mask_decoder)

    def warmup(self):
        dummy = np.zeros((480, 640, 3), dtype=np.uint8)
        box = np.array([100, 100, 400, 400], dtype=np.float32)

        try:
            self._set_image(dummy)
            _ = self._predict_box(box)
            self._info("NanoSAM warmup complete.")
        except Exception as e:  # noqa: BLE001
            self._warn(f"NanoSAM warmup failed: {e}")
            raise

    def _set_image(self, rgb_uint8: np.ndarray):
        self.predictor.set_image(PIL.Image.fromarray(rgb_uint8))

    def _predict_box(self, box_xyxy: np.ndarray) -> torch.Tensor:
        """Run NanoSAM for one XYXY box. Returns [H,W] float mask on CUDA.

        Point-label convention (from NanoSAM examples):
          2 = bounding box top-left, 3 = bounding box bottom-right
        """
        x0, y0, x1, y1 = [float(v) for v in box_xyxy]
        point_coords = np.array([[x0, y0], [x1, y1]], dtype=np.float32)
        point_labels = np.array([2, 3], dtype=np.float32)

        mask, _, _ = self.predictor.predict(point_coords, point_labels)
        return (mask[0, 0] > 0).float()

    @torch.inference_mode()
    def refine(self, rgb_np: np.ndarray, classes, boxes_xyxy, n_classes: int) -> torch.Tensor:
        """rgb_np: HxWx3 RGB uint8.
        classes: list[int], len K.
        boxes_xyxy: list[(x0,y0,x1,y1)], len K.

        Returns:
          refined: [n_classes,H,W] float32 on CUDA, values 0/1.
        """
        h, w = rgb_np.shape[:2]
        refined = torch.zeros((n_classes, h, w), device=self.dev, dtype=torch.float32)

        if not boxes_xyxy:
            return refined

        self._set_image(rgb_np)

        for cls, box in zip(classes, boxes_xyxy):
            if cls < 0 or cls >= n_classes:
                continue

            x0, y0, x1, y1 = box
            x0 = max(0, min(w - 1, float(x0)))
            x1 = max(0, min(w - 1, float(x1)))
            y0 = max(0, min(h - 1, float(y0)))
            y1 = max(0, min(h - 1, float(y1)))

            if x1 <= x0 or y1 <= y0:
                continue

            mask_t = self._predict_box(np.array([x0, y0, x1, y1], dtype=np.float32))
            refined[int(cls)] = torch.maximum(refined[int(cls)], mask_t)

        return refined
