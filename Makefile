# ptp-timestamping-unit master Makefile
#
#   make sim                 run every testbench
#   make sim-<tb>            run tb/tb_<tb>.sv, e.g. make sim-ptp_clock_core
#   make sim-<tb> WAVES=1    also dump build/tb_<tb>.vcd
#   make synth               Vivado OOC synthesis (Linux/Windows; see synth/)
#   make matlab              run the Simulink servo model + export golden vectors
#   make golden-diff         replay the vectors in RTL and diff (python report)

IVERILOG ?= iverilog
VVP      ?= vvp
VIVADO   ?= vivado
MATLAB   ?= $(shell command -v matlab 2>/dev/null || echo /Applications/MATLAB_R2026a.app/bin/matlab)

RTL_DIR   = rtl
TB_DIR    = tb
BUILD_DIR = build

RTL_SRCS  = $(wildcard $(RTL_DIR)/*.sv)
TB_SRCS   = $(wildcard $(TB_DIR)/tb_*.sv)
TB_LIBS   = $(filter-out $(TB_SRCS),$(wildcard $(TB_DIR)/*.sv))   # TB-only helper models
TB_NAMES  = $(patsubst $(TB_DIR)/tb_%.sv,%,$(TB_SRCS))

IVFLAGS   = -g2012 -Wall -Wno-timescale
PLUSARGS  = $(if $(WAVES),+WAVES,)

.PHONY: all sim synth matlab golden-diff clean
.SECONDARY:   # keep build/*.vvp

all: sim

# ── Simulation (Icarus) ──────────────────────────────────────────────────────

$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

$(BUILD_DIR)/tb_%.vvp: $(TB_DIR)/tb_%.sv $(RTL_SRCS) $(TB_LIBS) | $(BUILD_DIR)
	$(IVERILOG) $(IVFLAGS) -s tb_$* -o $@ $(RTL_SRCS) $(TB_LIBS) $<

sim-%: $(BUILD_DIR)/tb_%.vvp
	@$(VVP) -N $< $(PLUSARGS) | tee $(BUILD_DIR)/tb_$*.log
	@grep -q "TEST PASSED" $(BUILD_DIR)/tb_$*.log

sim: $(addprefix sim-,$(TB_NAMES))
	@echo "=== all testbenches passed: $(TB_NAMES) ==="

# ── Synthesis (Vivado, out-of-context) ──────────────────────────────────────

synth:
	cd synth && $(VIVADO) -mode batch -source ooc_synth.tcl

# ── MATLAB / Simulink servo model ───────────────────────────────────────────

matlab:
	cd matlab && $(MATLAB) -batch "run_servo_model"

golden-diff: sim-servo_golden
	python3 python/servo_diff.py

# ── Clean ───────────────────────────────────────────────────────────────────

clean:
	rm -rf $(BUILD_DIR) synth/vivado synth/*.log synth/*.jou matlab/slprj
