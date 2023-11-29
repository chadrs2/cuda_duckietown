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
run: all | data
	rm -f run_log.txt
	for i in {0..3}; do \
	  ./solution \
	    -e data/$$i/expect.pgm \
	    -i data/$$i/input.pgm \
	    -o data/$$i/output.pgm -t image \
	  | tee -a run_log.txt; \
	done

.PHONY: ncu
ncu: all | data
	rm -f run_log.txt
	for i in {3..3}; do \
	  ncu --mode=launch ./solution \
	    -e data/$$i/expect.pgm \
	    -i data/$$i/input.pgm \
	    -o data/$$i/output.pgm -t image \
	  | tee -a run_log.txt; \
	done
