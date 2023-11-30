################################################################################

# Sources and targets
# use TARGET for host only programs (no GPU)
# use NVTARGET for GPU programs
# TARGET = solution
NVTARGET = solution
MODULES = $(if $(wildcard solution.*),solution,template)
OBJECTS = $(addsuffix .o,$(MODULES))

################################################################################

include ../common.mak

# Include "data" as an order-only prerequisite to generate data
# e.g. run: all | data
.PHONY: run
run: all
	./solution | tee run_log.txt
