################################################################################

# Sources and targets
# use TARGET for host only programs (no GPU)
# use NVTARGET for GPU programs
TARGET = solution
# NVTARGET = solution
MODULES = $(if $(wildcard solution.*),solution,template)
OBJECTS = $(addsuffix .o,$(MODULES))

################################################################################

include ../common.mak
OPT += -fopenmp
LDFLAGS += -fopenmp

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
	for t in 1 2 {4..20..4}; do \
	  { echo "Max threads: $$t"; \
	  OMP_NUM_THREADS=$$t ./solution \
	    -e data/3/expect.pgm \
	    -i data/3/input.pgm \
	    -o data/3/output.pgm -t image; } \
	  | tee -a run_log.txt; \
	done

	python plotter.py
