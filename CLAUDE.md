# CLAUDE.md — PTP / IEEE-1588 Hardware Timestamping Unit

Context for Claude Code sessions working in this repo. See [README.md](README.md) for the
full write-up and [docs/REGMAP.md](docs/REGMAP.md) for the register map.

## What this is

A hardware timestamping and latency-measurement unit in SystemVerilog, built as a resume
project. Simulation and synthesis only, no board. Every block exists because the one next
to it needs it: the clock has to be read across domains, the timestamps have to become a
measurement, and the servo has to be modeled to know the clock converges.

## Blocks (all complete, all testbenches passing)

| Block | RTL | Testbench |
|---|---|---|
| 1588 clock core, Q8.32 DDS increment, ppb slewing | `rtl/ptp_clock_core.sv` | `tb/tb_ptp_clock_core.sv` |
| Timestamp capture + FIFO + drop counter | `rtl/ptp_ts_capture.sv`, `rtl/sync_fifo.sv` | `tb/tb_ptp_ts_capture.sv` |
| Coherent 80-bit time snapshot across clock domains | `rtl/ptp_time_snapshot.sv` | `tb/tb_ptp_time_snapshot.sv` |
| log2 latency histogram in BRAM | `rtl/ptp_latency_hist.sv` | `tb/tb_ptp_latency_hist.sv` |
| Register bridge / net regs / AXI-Lite / top | `rtl/cdc_bus_bridge.sv`, `rtl/ptp_net_regs.sv`, `rtl/axi_lite_regs.sv`, `rtl/ptp_tsu_top.sv` | `tb/tb_ptp_tsu_top.sv` |
| Simulink PI servo + bit-exact golden vectors | `matlab/` | `tb/tb_servo_golden.sv` |

Clocks: network domain 156.25 MHz (6.4 ns), AXI domain 100 MHz, asynchronous.

## Commands

```bash
make sim                      # all testbenches (Icarus); each prints TEST PASSED
make sim-ptp_clock_core       # one testbench; add WAVES=1 for a VCD in build/
make matlab                   # Simulink servo + golden vectors (MATLAB R2026a, Mac only)
make golden-diff              # replay vectors in RTL, then python/servo_diff.py
make synth                    # Vivado OOC synthesis + implementation (Windows VM)
```

`make sim-servo_golden` alone takes about 2 minutes 15 seconds; leave it out of quick loops.

## Current task

Vivado out-of-context synthesis. Script `synth/ooc_synth.tcl`, clocks in
`synth/ptp_tsu_clocks.xdc`, CDC exceptions in `synth/ptp_cdc.xdc` (sourced after
`synth_design`, so register names are post-synthesis `<sig>_reg[<bit>]`). Reports land in
`synth/reports/<part>/`, with `summary.txt` carrying Fmax, WNS, utilization and CDC counts.
Default part is `xc7z020clg400-1`, matching the PYNQ-Z2 used in the author's
fpga-crypto-feed-handler project.

Watch the network-clock timing. The histogram's stage 1 does a 49-bit subtract, a 35-bit
add, a compare, a 32-bit priority encoder and a barrel shift in one 6.4 ns cycle, and is
the likeliest place to miss timing. Fix by pipelining that stage, then rerun `make sim`.

After synthesis passes: update README with real numbers, commit, push.

## Environment

- Mac (primary): Icarus Verilog 12, MATLAB R2026a, Python 3 with numpy and matplotlib. No
  Vivado, no Verilator, no yosys.
- Windows 11 VM in Parallels: Vivado 2024.1 at `C:\Xilinx\Vivado\2024.1\bin\vivado.bat`.
  The Mac home folder is drive `Z:`. Work from `C:\work\...`, never a path with spaces, and
  never straight off the share; Vivado is unreliable with both.
- From the Mac, commands run inside the VM with
  `prlctl exec "Windows 11" cmd.exe /c "..."`.

## Conventions and gotchas

- One module per file, `default_nettype none`, header comment explaining the design problem
  and why the obvious approach fails. Testbenches are self-checking and print `TEST PASSED`.
- Testbenches drive stimulus on `negedge` and sample or compare on `posedge`, so checks see
  the same pre-edge values the DUT acts on. Getting this backwards caused a long false-alarm
  debug session on the capture unit.
- Icarus quirks: no `return` inside a `task`, no array-wide assignment from a list literal,
  and loop-body variables with initialisers behave as static, so hoist them to module scope.
- Golden vectors in `matlab/vectors/` are committed, so the RTL diff runs without MATLAB.
  Regenerating them must leave the file byte-identical unless the model actually changed.
- Commit after each completed step, with a message explaining what was verified.
