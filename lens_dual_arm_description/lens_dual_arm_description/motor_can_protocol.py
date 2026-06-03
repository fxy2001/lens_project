"""
电驱 CAN 协议打包（来自 `电驱通讯协议.pdf`）。

本模块的用途：在仿真运行时，把“要下发给真实电机”的控制指令实时编码成 CAN 报文并输出，
用于虚拟-现实对齐（Digital Twin 的 command side）。

当前实现重点：
- 标准包 MIT 控制（非广播）：CMD=0x0B，8 字节，速度/转矩/Kp/Kd 压缩为 10bit。
  见协议 3.3.1（非广播 MIT 下发）。
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable

import numpy as np


def _clamp(x: float, lo: float, hi: float) -> float:
    return float(min(max(x, lo), hi))


def util_float2Uint(x: float, x_min: float, x_max: float, bits: int) -> int:
    """
    协议中的 util_float2Uint：将 [x_min, x_max] 映射到 [0, 2^bits-1]。
    超界会夹紧。
    """
    if bits <= 0:
        raise ValueError("bits must be positive")
    x = _clamp(float(x), float(x_min), float(x_max))
    span = float(x_max) - float(x_min)
    if span <= 0:
        raise ValueError("x_max must be > x_min")
    return int(round((x - float(x_min)) * ((2**bits - 1) / span)))


@dataclass(frozen=True)
class MitRanges:
    pos_lower: float = -3.1415926
    pos_upper: float = 3.1415926
    vel_lower: float = -40.0
    vel_upper: float = 40.0
    tor_lower: float = -10.0
    tor_upper: float = 10.0
    kp_range: float = 300.0  # kp ∈ [0, kp_range]
    kd_range: float = 8.0    # kd ∈ [0, kd_range]


def pack_std_mit_non_broadcast(
    *,
    pos: float,
    vel: float,
    kp: float,
    kd: float,
    tor: float,
    ranges: MitRanges,
) -> bytes:
    """
    标准包（非广播）MIT 控制下发（CMD=0x0B），8 字节。

    协议位宽：
    - pos: 16bit
    - vel/kp/kd/tor: 10bit（因 CMD 占 1 字节）

    解包参考（协议代码）：
      vInt = (rx2<<2) | (rx3>>6)
      kpInt = ((rx3&0x3F)<<4) | (rx4>>4)
      kdInt = ((rx4&0xF)<<6) | (rx5>>2)
      tInt = ((rx5&0x3)<<8) | rx6
    """
    p = util_float2Uint(pos, ranges.pos_lower, ranges.pos_upper, 16) & 0xFFFF
    v = util_float2Uint(vel, ranges.vel_lower, ranges.vel_upper, 10) & 0x3FF
    kp_i = util_float2Uint(kp, 0.0, ranges.kp_range, 10) & 0x3FF
    kd_i = util_float2Uint(kd, 0.0, ranges.kd_range, 10) & 0x3FF
    t = util_float2Uint(tor, ranges.tor_lower, ranges.tor_upper, 10) & 0x3FF

    rx0 = (p >> 8) & 0xFF
    rx1 = p & 0xFF
    rx2 = (v >> 2) & 0xFF
    rx3 = ((v & 0x3) << 6) | ((kp_i >> 4) & 0x3F)
    rx4 = ((kp_i & 0xF) << 4) | ((kd_i >> 6) & 0xF)
    rx5 = ((kd_i & 0x3F) << 2) | ((t >> 8) & 0x3)
    rx6 = t & 0xFF

    return bytes([0x0B, rx0, rx1, rx2, rx3, rx4, rx5, rx6])


def format_can_frame(can_id: int, data: bytes) -> str:
    if not (0 <= int(can_id) <= 0x7FF):
        raise ValueError("CAN standard id must be 0..0x7FF")
    b = " ".join(f"{x:02X}" for x in data)
    return f"ID=0x{int(can_id):03X} DLC={len(data)} DATA={b}"


def format_multi_frames(frames: Iterable[tuple[int, bytes]]) -> str:
    return "\n".join(format_can_frame(i, d) for i, d in frames)


def parse_int_list_csv(s: str, n: int) -> list[int]:
    parts = [p.strip() for p in s.split(",") if p.strip()]
    if len(parts) != n:
        raise ValueError(f"Expected {n} ints, got {len(parts)}")
    return [int(p, 0) for p in parts]


def as_float_array(x: np.ndarray) -> np.ndarray:
    return np.asarray(x, dtype=float).reshape(-1)

