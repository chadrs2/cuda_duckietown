
PACKAGE = hpc-labs

# Cancel version control implicit rules
%:: %,v
%:: RCS/%
%:: RCS/%,v
%:: s.%
%:: SCCS/s.%
# Delete default suffixes
.SUFFIXES:

APPS = libhtk DeviceQuery VectorAdd SimpleMatMul TiledMatMul HotPlate_CUDA HotPlate_CPP HotPlate_OMP HotPlate_LIB PyTorch

.PHONY: all
all .DEFAULT:
ifneq ($(filter clean,$(MAKECMDGOALS)),)
	$(MAKE) -C libhtk $@
endif
	$(MAKE) -C DeviceQuery $@
	$(MAKE) -C VectorAdd $@
	$(MAKE) -C SimpleMatMul $@
	$(MAKE) -C TiledMatMul $@
	$(MAKE) -C HotPlate_CUDA $@
	$(MAKE) -C HotPlate_CPP $@
	$(MAKE) -C HotPlate_OMP $@
	$(MAKE) -C HotPlate_LIB $@
	$(MAKE) -C PyTorch $@

.PHONY: $(APPS)
$(APPS):
	$(MAKE) -C $@ all

.PHONY: help
help:
	@echo "Applications can also be built individually by changing"
	@echo "directories and typing 'make' within the subdirectory."
	@echo ""
	@echo "Targets:"
	@echo "  <app>  - Build specific application:"
	@echo "           $(APPS)"
	@echo "           e.g., make DeviceQuery"
	@echo "  all    - Build all applications"
	@echo "  clean  - Remove files for all applications"
	@echo "  dist   - Package the release for distribution (tar)"
	@echo "  run    - Build and run all applications"
	@echo "  submit - Create submission files for all applications"

.PHONY: dist
dist:
	$(MAKE) clean
	rm -f ../$(PACKAGE).tgz *.zip
	tar --transform 's,^,$(PACKAGE)/,' $(if $(NO_SOL),--exclude='solution.*',) -czf ../$(PACKAGE).tgz *
