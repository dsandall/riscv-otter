VLOG = $(wildcard *.sv)
TOP = OTTER_Wrapper_Programmable
OTHER_SRC = otter_memory.mem
XDC    = basys3.xdc
F4PGA_JSON = flow_basys3.json

include $(SHORTSHIFT_INSTALL)/scripts/Makefile.mk 
