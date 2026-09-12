# Register map — `ptp_tsu_top` (AXI4-Lite, 32-bit, 4 KiB)

Offsets `0x000–0x0FF` live in the AXI clock domain and respond immediately.
Offsets `0x100` and above live in the **network clock domain** and are reached
through `cdc_bus_bridge`; each access costs one handshake round trip
(≈ 2 network + 2 AXI cycles each way, ~10–15 AXI cycles total).

## AXI domain

| offset | name | access | description |
|---|---|---|---|
| `0x000` | `ID` | RO | `0x50545355` ("PTSU") |
| `0x004` | `VERSION` | RO | `0x00010000` |
| `0x008` | `CTRL` | WO | bit 0 `SNAP_REQ`: take a coherent time snapshot (self-clearing) |
| `0x00C` | `STATUS` | RO | bit 0 `snap_busy`, bit 1 `bridge_busy` |
| `0x010` | `SNAP_NS` | RO | nanoseconds of the last snapshot |
| `0x014` | `SNAP_SEC_LO` | RO | seconds[31:0] |
| `0x018` | `SNAP_SEC_HI` | RO | seconds[47:32] |

`SNAP_*` are latched together on the snapshot's completion, so reading them as
three separate words is always self-consistent. Software: write `CTRL=1`, poll
`STATUS.snap_busy == 0`, read the three words.

## Network domain (via bridge)

| offset | name | access | description |
|---|---|---|---|
| `0x100` | `NOM_INCR_LO` | RW | nominal increment, fractional bits [31:0] (reset `0x66666666`) |
| `0x104` | `NOM_INCR_HI` | RW | nominal increment, integer ns bits [7:0] (reset `0x6`) |
| `0x108` | `FREQ_ADJ` | RW | signed addend to the increment, LSB = 2⁻³² ns/cycle (0.036 ppb at 6.4 ns) |
| `0x10C` | `ADJ_SEC` | RW | signed seconds part of the next phase step (staged) |
| `0x110` | `ADJ_NS` | WO | signed ns part; **writing applies** `ADJ_SEC:ADJ_NS` once. `|ADJ_NS| < 1e9` |
| `0x114` | `SET_SEC_LO` | RW | absolute time to load, seconds[31:0] (staged) |
| `0x118` | `SET_SEC_HI` | RW | seconds[47:32] (staged) |
| `0x11C` | `SET_NS` | WO | ns; **writing loads** `SET_SEC:SET_NS` and clears the fraction |
| `0x120` | `HIST_COUNT` | RO | samples binned (saturating) |
| `0x124` | `HIST_SUM_LO` | RO | Σ delta, bits [31:0] (ns) |
| `0x128` | `HIST_SUM_HI` | RO | Σ delta, bits [63:32] |
| `0x12C` | `HIST_MIN` | RO | min delta (ns), `0xFFFFFFFF` when empty |
| `0x130` | `HIST_MAX` | RO | max delta (ns) |
| `0x134` | `HIST_NEG` | RO | pairs with `t_out < t_in` (not binned) |
| `0x138` | `DROP_IN` | RO | ingress strobes dropped because the capture FIFO was full |
| `0x13C` | `DROP_OUT` | RO | egress strobes dropped |
| `0x140` | `FIFO_LEVELS` | RO | [31:16] egress FIFO count, [15:0] ingress FIFO count |
| `0x144` | `NET_CTRL` | WO | bit 0 `HIST_CLEAR` (zero bins + stats), bit 1 `DROP_CLEAR` |
| `0x148` | `NET_STATUS` | RO | bit 0 `hist_busy` (clear in progress) |
| `0x800–0x9FC` | `HIST_BIN[0..127]` | RO | bin counters, `SUB_BITS = 2` → 128 bins |

### Bin index → latency range

Dense HdrHistogram-style log₂ binning with `SUB_BITS = 2` sub-bins per octave:

```
delta < 8 :  bin = delta                                   (linear, bins 0..7)
otherwise :  msb = highest set bit of delta, sub = the 2 bits below it
             bin = 4*msb + sub - 4                          (bins 8..123)

lower edge:  lo(k) = k                          for k < 8
             lo(k) = (4 + ((k-8) & 3)) << (((k-8) >> 2) + 1)   otherwise
```

so the edges run 0,1,…,7, 8,10,12,14, 16,20,24,28, 32,40,48,56, … ns (25 %
relative width per bin) up to 2³² ns; 124 bins are used of the 128 available.
Mean = `HIST_SUM / HIST_COUNT`.

Conversion helpers: [python/ptp_tsu.py](../python/ptp_tsu.py).
