export PATH := /home/shay/a/ece270/bin:$(PATH)
export LD_LIBRARY_PATH := /home/shay/a/ece270/lib:$(LD_LIBRARY_PATH)

YOSYS=yosys
NEXTPNR=nextpnr-ice40
SHELL=bash

PROJ    = 32bitcpu
PINMAP  = pinmap.pcf
CELLS	= support/cells_map_timing.v support/cells_sim_timing.v
SRC     = top.sv
TB	    = tb.sv
ICE     = ice40hx8k.sv
UART    = support/uart.v support/uart_tx.v support/uart_rx.v
FILES   = $(ICE) $(SRC) $(UART)
BUILD   = ./build
TRACE   = sim.vcd

DEVICE  = 8k
TIMEDEV = hx8k
FOOTPRINT = ct256

all: cram

#########################
# Compile, synthesis, place and route, and bitstream generation for ice40 FPGA
$(BUILD)/$(PROJ).json : $(ICE) $(SRC) $(PINMAP) Makefile
	# lint with Verilator
	verilator --lint-only --top-module top $(SRC)
	# if build folder doesn't exist, create it
	mkdir -p $(BUILD)
	# synthesize using Yosys
	$(YOSYS) -p "read_verilog -sv -noblackbox $(FILES); synth_ice40 -top ice40hx8k -json $(BUILD)/$(PROJ).json"

$(BUILD)/$(PROJ).asc : $(BUILD)/$(PROJ).json
	# Place and route using nextpnr
	$(NEXTPNR) --hx8k --package ct256 --pcf $(PINMAP) --asc $(BUILD)/$(PROJ).asc --json $(BUILD)/$(PROJ).json 2> >(sed -e 's/^.* 0 errors$$//' -e '/^Info:/d' -e '/^[ ]*$$/d' 1>&2)

$(BUILD)/$(PROJ).bin : $(BUILD)/$(PROJ).asc
	# Convert to bitstream using IcePack
	icepack $(BUILD)/$(PROJ).asc $(BUILD)/$(PROJ).bin

#########################
# Run a testbench
sim: $(SRC)
	# if build folder doesn't exist, create it
	mkdir -p $(BUILD)
	# synthesize with yosys to cell-level Verilog
	$(YOSYS) -p "read_verilog -sv -noblackbox $(SRC); synth_ice40; write_verilog $(BUILD)/$(PROJ).v"
	# run simulation
	iverilog -g2012 $(CELLS) $(BUILD)/$(PROJ).v $(TB) -o $(BUILD)/$(PROJ)
	vvp $(BUILD)/$(PROJ)
	gtkwave $(TRACE)

verify_%: $(SRC) %.sv tb_%.sv
	# if build folder doesn't exist, create it
	mkdir -p $(BUILD)
	# synthesize with yosys to cell-level Verilog
	$(YOSYS) -p "read_verilog -sv -noblackbox $*.sv; synth_ice40 -top $*; write_verilog $(BUILD)/$(PROJ).v"
	# run simulation
	iverilog -g2012 $(CELLS) $(BUILD)/$(PROJ).v tb_$*.sv -o $(BUILD)/$(PROJ)
	vvp $(BUILD)/$(PROJ)
	gtkwave $(TRACE)

# source_sim: $(SRC)
# 	# if build folder doesn't exist, create it
# 	mkdir -p $(BUILD)
# 	# run simulation
# 	iverilog -g2012 $(SRC) $(TB) -o $(BUILD)/$(PROJ)
# 	vvp $(BUILD)/$(PROJ)
# 	gtkwave $(TRACE)

# mapped_sim: $(SRC)
# 	# if build folder doesn't exist, create it
# 	mkdir -p $(BUILD)
# 	# synthesize with yosys to cell-level Verilog
# 	$(YOSYS) -p "read_verilog -sv -noblackbox $(SRC); synth_ice40; write_verilog $(BUILD)/$(PROJ).v"
# 	# run simulation
# 	iverilog -g2012 $(CELLS) $(BUILD)/$(PROJ).v $(TB) -o $(BUILD)/$(PROJ)
# 	vvp $(BUILD)/$(PROJ)
# 	gtkwave $(TRACE)

#########################
# ice40 Specific Targets
check: $(CHK)
	iceprog -S $(CHK)
	
demo:
	mkdir -p $(BUILD)
	$(YOSYS) -p "read_verilog -sv $(ICE) support/lock_bb_top.sv blackbox_lock.v $(UART); synth_ice40 -top ice40hx8k; write_json $(BUILD)/$(PROJ).json"
	$(NEXTPNR) --hx8k --package ct256 --pcf $(PINMAP) --asc $(BUILD)/$(PROJ).asc --json $(BUILD)/$(PROJ).json 2> >(sed -e 's/^.* 0 errors$$//' -e '/^Info:/d' -e '/^[ ]*$$/d' 1>&2)
	icepack $(BUILD)/$(PROJ).asc $(BUILD)/$(PROJ).bin
	iceprog $(BUILD)/$(PROJ).bin

flash: $(BUILD)/$(PROJ).bin
	iceprog $(BUILD)/$(PROJ).bin

cram: $(BUILD)/$(PROJ).bin
	iceprog -S $(BUILD)/$(PROJ).bin

time: $(BUILD)/$(PROJ).asc
	icetime -p $(PINMAP) -P $(FOOTPRINT) -d $(TIMEDEV) $<

#########################
# Clean Up
clean:
	rm -rf build/ *.fst *.vcd verilog.log abc.history
