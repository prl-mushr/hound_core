"""Cross-process CUDA tensor handoff.

Preferred: PyTorch CUDA IPC (``torch.multiprocessing.reductions``). On Jetson /
Tegra this typically returns ``cudaErrorNotSupported`` — probe once and use the
Arrow float16 path instead (a few ms; fine next to SAM ~100ms).
"""

from __future__ import annotations

import collections
import pickle
from typing import Deque

import numpy as np
import pyarrow as pa
import torch
from torch.multiprocessing import reductions


# Sender must keep shared storages alive until the peer has opened + cloned.
# Keep enough shared storages alive for pipelined dora hops (queue_size ~3–8).
_RETAIN: Deque[torch.Tensor] = collections.deque(maxlen=16)

# Cached probe result: True / False / None (not yet probed).
_IPC_OK: bool | None = None


def cuda_ipc_supported(device: str | torch.device = "cuda") -> bool:
    """One-shot probe. Jetson Orin usually returns False (cudaErrorNotSupported)."""
    global _IPC_OK
    if _IPC_OK is not None:
        return _IPC_OK
    if not torch.cuda.is_available():
        _IPC_OK = False
        return False
    try:
        t = torch.zeros(4, device=device, dtype=torch.float32)
        rebuild, args = reductions.reduce_tensor(t.contiguous())
        if rebuild is not reductions.rebuild_cuda_tensor:
            _IPC_OK = False
            return False
        # Force the CUDA IPC handle creation / open round-trip in-process.
        shared = reductions.rebuild_cuda_tensor(*args)
        _ = shared.clone()
        del shared, t
        torch.cuda.synchronize()
        _IPC_OK = True
    except Exception:
        _IPC_OK = False
    return _IPC_OK


def pack_cuda_tensor(tensor: torch.Tensor) -> tuple[pa.Array, dict, str]:
    """GPU tensor -> (Arrow handle payload, meta extras, mode tag)."""
    if not tensor.is_cuda:
        raise ValueError("pack_cuda_tensor expects a CUDA tensor")
    if not cuda_ipc_supported(tensor.device):
        raise RuntimeError("cudaErrorNotSupported: CUDA IPC unavailable on this GPU")
    t = tensor.detach().contiguous()
    rebuild, args = reductions.reduce_tensor(t)
    if rebuild is not reductions.rebuild_cuda_tensor:
        raise RuntimeError(
            f"unexpected reduce rebuild {getattr(rebuild, '__name__', rebuild)}"
        )
    payload = pickle.dumps(args, protocol=pickle.HIGHEST_PROTOCOL)
    _RETAIN.append(t)
    meta = {
        "dtype": str(t.dtype).removeprefix("torch."),
        "shape": [int(x) for x in t.shape],
        "stride": [int(x) for x in t.stride()],
    }
    return pa.array(bytearray(payload), type=pa.uint8()), meta, "torch_cuda_ipc"


def unpack_cuda_tensor(arrow_value, meta: dict | None = None) -> torch.Tensor:
    """Arrow handle payload -> owned CUDA tensor (cloned off the IPC mapping)."""
    raw = arrow_value
    if hasattr(raw, "to_numpy"):
        buf = bytes(raw.to_numpy())
    elif isinstance(raw, (bytes, bytearray, memoryview)):
        buf = bytes(raw)
    else:
        buf = bytes(raw)
    args = pickle.loads(buf)
    shared = reductions.rebuild_cuda_tensor(*args)
    owned = shared.detach().clone()
    del shared
    return owned


def pack_arrow_f16(tensor: torch.Tensor) -> pa.Array:
    """GPU tensor -> Arrow float16 (pinned host staging when possible)."""
    t = tensor.detach().contiguous().to(dtype=torch.float16)
    # page-locked host buffer cuts D2H latency a bit on Orin
    host = torch.empty(t.shape, dtype=torch.float16, pin_memory=True)
    host.copy_(t, non_blocking=True)
    torch.cuda.synchronize()
    return pa.array(host.numpy().reshape(-1), type=pa.float16())


def unpack_arrow_f16(
    arrow_value,
    shape: tuple[int, ...],
    *,
    non_blocking: bool = False,
) -> torch.Tensor:
    """Arrow float16 -> CUDA float32 tensor matching ``shape``.

    With ``non_blocking=True``, the H2D copy uses the current CUDA stream
    (pair with ``torch.cuda.stream`` + a later synchronize/wait).
    """
    flat = np.ascontiguousarray(arrow_value.to_numpy(), dtype=np.float16)
    host = torch.from_numpy(flat)
    return (
        host.to(device="cuda", dtype=torch.float32, non_blocking=non_blocking)
        .reshape(*shape)
        .contiguous()
    )
