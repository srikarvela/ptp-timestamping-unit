"""Register-level helpers for the PTP timestamping unit (see docs/REGMAP.md).

Pure-Python conversions used by the golden-vector tooling and by any host
driver (PYNQ/MMIO or otherwise) that talks to ptp_tsu_top.
"""

from __future__ import annotations

NS_PER_SEC = 1_000_000_000
FRAC_BITS = 32
SUB_BITS = 2
NBINS = 32 << SUB_BITS

# --- register offsets ---------------------------------------------------------
ID, VERSION, CTRL, STATUS = 0x000, 0x004, 0x008, 0x00C
SNAP_NS, SNAP_SEC_LO, SNAP_SEC_HI = 0x010, 0x014, 0x018
NOM_INCR_LO, NOM_INCR_HI, FREQ_ADJ = 0x100, 0x104, 0x108
ADJ_SEC, ADJ_NS = 0x10C, 0x110
SET_SEC_LO, SET_SEC_HI, SET_NS = 0x114, 0x118, 0x11C
HIST_COUNT, HIST_SUM_LO, HIST_SUM_HI = 0x120, 0x124, 0x128
HIST_MIN, HIST_MAX, HIST_NEG = 0x12C, 0x130, 0x134
DROP_IN, DROP_OUT, FIFO_LEVELS = 0x138, 0x13C, 0x140
NET_CTRL, NET_STATUS, HIST_BIN = 0x144, 0x148, 0x800


def period_to_incr(period_ns: float) -> int:
    """Nominal period in ns -> Q8.32 increment word (6.4 ns -> 0x666666666)."""
    return int(period_ns * (1 << FRAC_BITS))


def ppb_to_freq_adj(ppb: float, period_ns: float = 6.4) -> int:
    """Frequency offset in parts-per-billion -> signed FREQ_ADJ register value.

    1 ppb of a 6.4 ns period is 6.4e-9 ns/cycle = 27.49 LSB, so the register
    resolves 0.036 ppb.
    """
    return int(round(ppb * 1e-9 * period_ns * (1 << FRAC_BITS)))


def freq_adj_to_ppb(adj: int, period_ns: float = 6.4) -> float:
    return adj / ((1 << FRAC_BITS) * period_ns) * 1e9


def split_offset(offset_ns: int) -> tuple[int, int]:
    """Signed total-ns offset -> (ADJ_SEC, ADJ_NS) with |ADJ_NS| < 1e9."""
    sec = int(offset_ns / NS_PER_SEC)           # truncate toward zero
    ns = offset_ns - sec * NS_PER_SEC
    return sec, ns


LIN_N = 1 << (SUB_BITS + 1)                       # linear region: bins 0..7 = deltas 0..7
OFFSET = ((SUB_BITS + 1) << SUB_BITS) - LIN_N     # index shift for the log region
NBINS_USED = LIN_N + (32 - SUB_BITS - 1) * (1 << SUB_BITS)


def bin_index(delta_ns: int) -> int:
    """Bin index for a non-negative latency (same formula as the RTL)."""
    d = min(int(delta_ns), 0xFFFF_FFFF)
    if d < LIN_N:
        return d
    msb = d.bit_length() - 1
    sub = (d >> (msb - SUB_BITS)) & ((1 << SUB_BITS) - 1)
    return (msb << SUB_BITS) + sub - OFFSET


def bin_lo(k: int) -> int:
    """Lower edge (inclusive, ns) of bin k."""
    if k < LIN_N:
        return k
    j = k - LIN_N
    msb = (j >> SUB_BITS) + SUB_BITS + 1
    sub = j & ((1 << SUB_BITS) - 1)
    return ((1 << SUB_BITS) + sub) << (msb - SUB_BITS)


def bin_edges() -> list[tuple[int, int]]:
    """[(lo, hi_inclusive)] for every bin."""
    los = [bin_lo(k) for k in range(NBINS_USED)] + [1 << 32]
    return [(los[k], los[k + 1] - 1) for k in range(NBINS_USED)]


if __name__ == "__main__":
    print(f"nom_incr(6.4 ns)      = 0x{period_to_incr(6.4):X}")
    print(f"freq_adj(+100 ppm)    = {ppb_to_freq_adj(100_000)}")
    print(f"freq_adj(+1 ppb)      = {ppb_to_freq_adj(1)}")
    print(f"resolution            = {freq_adj_to_ppb(1):.4f} ppb/LSB")
    print("first bin edges       =", [lo for lo, _ in bin_edges()[:20]])
    # self-check: every bin edge maps back to its own index, densely
    for k, (lo, hi) in enumerate(bin_edges()):
        assert bin_index(lo) == k and bin_index(hi) == k, (k, lo, hi)
    print(f"{NBINS_USED} dense bins verified (of {NBINS} available)")
