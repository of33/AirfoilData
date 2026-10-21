# Makefile for the AirfoilData example program
#
# Targets:
#   make / make example   build the example into bin/example
#   make lib              (optional) archive the library objects from
#                         build/ as lib/libAirfoilData.a
# The example links the two locally compiled object files directly.
# The .a archive is only an optional convenience for compatible
# Unix-like toolchains.
#   make test             run all tests except valgrind (harness in tests/)
#   make plot             run the example + render four PNGs with gnuplot
#   make clean            remove all build products (bin/ + build/ + lib/ +
#                         tests/build + plots)

CXX      := g++
CXXFLAGS := -std=c++17 -Wall -Wextra -Isrc
GNUPLOT  ?= gnuplot
AR      := ar
ARFLAGS := rcs

CORE     := src/AirfoilData.cpp src/AkimaInterpolation.cpp
HEADERS  := src/AirfoilData.h src/AkimaInterpolation.h
EXAMPLE  := example.cpp

# Executable suffix: GNU make on Windows (OS=Windows_NT, e.g. MinGW/MSYS2)
# needs .exe; Unix leaves it empty.
ifeq ($(OS),Windows_NT)
EXE      := .exe
else
EXE      :=
endif

BINDIR   := bin
TARGET   := $(BINDIR)/example$(EXE)
PLOTDIR  := plot
OBJDIR   := build
LIBDIR   := lib
LIB      := $(LIBDIR)/libAirfoilData.a

OBJS := $(OBJDIR)/AirfoilData.o $(OBJDIR)/AkimaInterpolation.o

.PHONY: all example objects lib test plot clean

all: example

example: $(TARGET)

lib: $(LIB)

objects: $(OBJS)

$(OBJDIR)/%.o: src/%.cpp $(HEADERS) | $(OBJDIR)
	$(CXX) $(CXXFLAGS) -c -o $@ $<

$(LIB): $(OBJS)
	@mkdir -p $(LIBDIR)
	$(AR) $(ARFLAGS) $@ $(OBJS)

$(TARGET): $(EXAMPLE) $(OBJS) $(HEADERS) | $(BINDIR)
	$(CXX) $(CXXFLAGS) -o $@ $(EXAMPLE) $(OBJS)

$(BINDIR):
	mkdir -p $(BINDIR)

$(OBJDIR):
	mkdir -p $(OBJDIR)

test:
	$(MAKE) -C tests check

# Runs the example (writes example_*.dat in the root directory) and renders
# four PNGs in plot/ from it. Without gnuplot: only the .dat files.
plot: example
	@mkdir -p $(PLOTDIR)
	./$(TARGET)
	@if command -v $(GNUPLOT) >/dev/null 2>&1; then \
	    $(GNUPLOT) $(PLOTDIR)/plot.gnu; \
	    echo "Plots created: $(PLOTDIR)/example_*.png"; \
	else \
	    echo "gnuplot not found - only example_*.dat written"; \
	fi

clean:
	rm -rf $(BINDIR) $(OBJDIR) $(LIBDIR)
	rm -f example_*.dat $(PLOTDIR)/*.png
	$(MAKE) -C tests clean
