##########################################
# Options
##########################################
HTK_LIB_PATH=$(CURDIR)/lib
HTK_SRC_PATH=$(CURDIR)
CXX=g++

##########################################
##########################################

DEFINES += -DHTK_USE_JSON11=0
# DEFINES += -DHTK_USE_COURSERA

##########################################
##########################################

CXX_FLAGS=-fpic -O3 -std=c++11 -MMD -I. -I$(HTK_SRC_PATH) # -I/usr/local/cuda/include -L/usr/local/cuda/lib64 
LIBS=-lm -std=c++11 -L$(HTK_LIB_PATH) # -lcuda

##########################################
##########################################

UNAME_S := $(shell uname -s)
ifeq ($(UNAME_S),Linux)
    LIBS += -lrt
endif

##########################################
##########################################

SOURCES := $(shell find $(HTK_SRC_PATH) ! -name "*_test.cpp" ! -name json11.cpp -name "*.cpp")
TESTS :=  $(shell find $(HTK_SRC_PATH) -name "*_test.cpp")

OBJECTS = $(SOURCES:.cpp=.o)

##############################################
# OUTPUT
##############################################

.SUFFIXES: .o .cpp

.PHONY: all
all: libhtk.a

.cpp.o:
	$(CXX) $(DEFINES) $(CXX_FLAGS) -c -o $@ $<

libhtk.so: $(OBJECTS)
	mkdir -p $(HTK_LIB_PATH)
	$(CXX) -shared -o $(HTK_LIB_PATH)/$@ $(OBJECTS)

libhtk.a: $(OBJECTS)
	mkdir -p $(HTK_LIB_PATH)
	ar rcs -o $(HTK_LIB_PATH)/$@ $(OBJECTS)

$(OBJECTS): $(MAKEFILE_LIST) # rebuild if MAKEFILEs change

.PHONY: test
test: libhtk.a
	$(CXX) $(DEFINES) $(CXX_FLAGS) -o $@ $(TESTS) -lhtk $(LIBS)

.PHONY: clean
clean:
	rm -rf *.o *.d lib test test-dataset

-include $(OBJECTS:.o=.d) # Establish module specific dependencies
