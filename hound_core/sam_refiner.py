"""Shared EfficientViT-SAM refinement helper.

Used by both the in-process refine path of clipseg_mask_node and the standalone
sam_refine_node, so the SAM logic (build + compile + warmup + batched box
refinement) lives in exactly one place.
"""

import numpy as np
import torch


class SamRefiner:
    """Loads EfficientViT-SAM (optionally torch.compiled), and turns box prompts
    into crisp per-class masks via a single batched decoder call."""

    def __init__(self, device, variant="l0", ckpt="",
                 compile_mode="reduce-overhead", logger=None):
        self.dev = device
        self.variant = variant
        self.ckpt = ckpt
        self.compile_mode = compile_mode
        self.log = logger
        self.predictor = self._build()
        self.warmup()

    def _info(self, m):
        if self.log:
            self.log.info(m)

    def _warn(self, m):
        if self.log:
            self.log.warn(m)

    def _build(self):
        from efficientvit.sam_model_zoo import create_efficientvit_sam_model
        from efficientvit.models.efficientvit.sam import EfficientViTSamPredictor

        sam = create_efficientvit_sam_model(
            name=f"efficientvit-sam-{self.variant}",
            pretrained=True,
            weight_url=self.ckpt or None,
        ).to(self.dev).eval()
        self._sam_model = sam
        self._encoder_eager = sam.image_encoder
        if self.compile_mode and self.compile_mode.lower() != "none":
            try:
                sam.image_encoder = torch.compile(
                    sam.image_encoder, mode=self.compile_mode, dynamic=False
                )
                self._info(f"torch.compile(SAM image_encoder, mode='{self.compile_mode}')")
            except Exception as e:  # noqa: BLE001
                self._warn(f"SAM compile failed ({e}); running eager")
        return EfficientViTSamPredictor(sam)

    def warmup(self, iters=10):
        # reduce-overhead / max-autotune need several iters to compile + capture.
        dummy = np.zeros((480, 640, 3), dtype=np.uint8)
        box = np.array([100, 100, 400, 400], dtype=np.float32)

        def run():
            with torch.inference_mode(), torch.autocast("cuda", dtype=torch.float16):
                for _ in range(iters):
                    self.predictor.set_image(dummy)
                    self.predictor.predict(box=box, multimask_output=False)
            torch.cuda.synchronize()

        try:
            run()
        except Exception as e:  # noqa: BLE001 -- compile (e.g. max-autotune) can fail
            if self._sam_model.image_encoder is not self._encoder_eager:
                self._warn(f"SAM compile/warmup failed ({e}); falling back to eager")
                self._sam_model.image_encoder = self._encoder_eager
                run()
            else:
                raise
        self._info("SAM warmup/compile complete.")

    @torch.inference_mode()
    def refine(self, rgb_np: np.ndarray, classes, boxes_xyxy, n_classes: int) -> torch.Tensor:
        """rgb_np: HxWx3 RGB uint8. classes: list[int] len K. boxes_xyxy: list of
        (x0,y0,x1,y1) len K. Returns per-class binary masks [n_classes,H,W] (float
        0/1) on device, OR'ing all boxes of the same class together."""
        h, w = rgb_np.shape[:2]
        refined = torch.zeros((n_classes, h, w), device=self.dev)
        if not boxes_xyxy:
            return refined

        boxes = torch.tensor(boxes_xyxy, dtype=torch.float32, device=self.dev)
        with torch.autocast("cuda", dtype=torch.float16):
            self.predictor.set_image(rgb_np)                # encoder once/frame
            tboxes = self.predictor.apply_boxes_torch(boxes)
            masks, _, _ = self.predictor.predict_torch(      # ALL boxes in one pass
                point_coords=None, point_labels=None,
                boxes=tboxes, multimask_output=False,
            )

        masks = masks[:, 0].to(torch.float32)               # [K,H,W]
        for k, i in enumerate(classes):
            if 0 <= i < n_classes:
                refined[i] = torch.maximum(refined[i], masks[k])
        return refined
