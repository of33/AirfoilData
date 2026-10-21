# Test harness for AirfoilData: golden generator and test program.
#
# Layout:
#   src/        harness sources (golden_generator.cpp, test_airfoildata.cpp,
#               harness_common.{cpp,h})
#   data/       reference (golden.txt) and test profile (NACA4618.dat)
#   build/bin/  compiled programs
#   build/cov/  coverage artifacts (.gcno/.gcda)
#   build/obj/  analyzer objects (-fanalyzer)
#
# Targets:
#   make golden-gcc / make golden-clang   generate the golden reference
#   make update-golden                    intentionally regenerate the
#                                         committed golden reference
#   make test-gcc   / make test-clang     check against the golden reference
#   make san-gcc    / make san-clang      test run with address/undefined sanitizer
#   make san-full-gcc                     like san-gcc, plus full UBSan expansion
#   make dbg-gcc                          test run with debug libstdc++ (bounds/iterators)
#   make coverage                         gcov method coverage of the core sources
#   make analyzer                         g++ -fanalyzer (static analysis)
#   make tidy                             clang-tidy (clang-analyzer checks)
#   make memcheck                         valgrind memcheck on the gcc test
#   make sharp                            all sharpeners (san-full/dbg/coverage/analyzer/tidy/memcheck)
#   make strict                           like test-gcc, plus -Werror
#   make all                              run the main tests against the committed golden
#   make full                             everything: check + memcheck + strict
#   make clean
#
# -Werror is the default (STRICT ?= -Werror); use `make STRICT= ...`
# to switch it off for a single run

CXX_GCC   := g++
CXX_CLANG := clang++

WARNINGS  := -Wall -Wextra -Wpedantic -Wshadow -Wconversion -Wsign-conversion -Wold-style-cast -Wnon-virtual-dtor -Woverloaded-virtual
STRICT    ?= -Werror
CXXFLAGS  := -std=c++17 $(WARNINGS) $(STRICT) -I../src -Isrc
SANFLAGS  := -fsanitize=address,undefined -fno-omit-frame-pointer -g -fno-sanitize-recover=all
# Clang additionally enables the implicit-conversion sanitizer (available since
# Clang 3.x; the group's default checkers: int-truncation/sign-change)
SAN_CLANG_EXTRA := -fsanitize=implicit-conversion

# Full UBSan expansion beyond the standard "undefined" group
UBSAN_FULL := -fsanitize=bounds,bounds-strict,float-cast-overflow,float-divide-by-zero,integer-divide-by-zero,signed-integer-overflow,pointer-overflow,alignment,null,unreachable,shift
# Debug mode of libstdc++ (container bounds/iterator checks)
DBGFLAGS  := -D_GLIBCXX_DEBUG -D_GLIBCXX_ASSERTIONS
# Coverage (gcov)
COVFLAGS  := --coverage -g -O0

# Core sources
CORE_SRC  := ../src/AirfoilData.cpp ../src/AkimaInterpolation.cpp
# Harness sources
SRCDIR    := src
HARNESS   := $(SRCDIR)/harness_common.cpp
HARNESS_H := $(SRCDIR)/harness_common.h
GOLDEN_MAIN := $(SRCDIR)/golden_generator.cpp
TEST_MAIN   := $(SRCDIR)/test_airfoildata.cpp
# Input/output data
GOLDEN    := data/golden.txt
PROFILE   := data/NACA4618.dat

BINDIR    := build/bin
COVDIR    := build/cov
ANADIR    := build/obj

GOLDEN_GCC   := $(BINDIR)/golden_generator_gcc
GOLDEN_CLANG := $(BINDIR)/golden_generator_clang
TEST_GCC     := $(BINDIR)/test_airfoildata_gcc
TEST_CLANG   := $(BINDIR)/test_airfoildata_clang
SAN_GCC      := $(BINDIR)/test_airfoildata_san_gcc
SAN_CLANG    := $(BINDIR)/test_airfoildata_san_clang
SAN_FULL     := $(BINDIR)/test_airfoildata_san_full
TEST_DBG     := $(BINDIR)/test_airfoildata_dbg
TEST_COV     := $(BINDIR)/test_airfoildata_cov

COV_OBJS := $(COVDIR)/AirfoilData.o $(COVDIR)/AkimaInterpolation.o $(COVDIR)/harness_common.o $(COVDIR)/test_airfoildata.o

# Sanitizer runtime probes: some platforms (e.g. MSYS2 UCRT64) ship gcc
# without libasan/libubsan and clang without the matching compiler-rt
# files. The sanitizer targets then report SKIP instead of failing.
ASAN_GCC := $(shell printf 'int main(void){return 0;}' >san_probe.c && \
    $(CXX_GCC) -fsanitize=address,undefined san_probe.c -o san_probe 2>/dev/null && echo yes; \
    rm -f san_probe.c san_probe san_probe.exe)
ASAN_CLANG := $(shell printf 'int main(void){return 0;}' >san_probe.c && \
    $(CXX_CLANG) -fsanitize=address,undefined san_probe.c -o san_probe 2>/dev/null && echo yes; \
    rm -f san_probe.c san_probe san_probe.exe)

.PHONY: all full check update-golden golden-gcc golden-clang test-gcc test-clang san-gcc san-clang san-full-gcc dbg-gcc coverage analyzer tidy memcheck sharp strict clean

all: test-gcc test-clang san-gcc san-clang

# Everything: the full check suite (golden comparisons, sanitizers, debug
# libstdc++, coverage, analyzer, clang-tidy) plus valgrind memcheck and
# the strict -Werror build.
full: check memcheck strict

# All tests except valgrind/memcheck; collects failures and prints a
# final summary line. The committed golden reference is used unchanged;
# regenerate it deliberately with 'make update-golden' (or golden-gcc).
CHECK_STEPS := test-gcc test-clang san-gcc san-clang san-full-gcc dbg-gcc coverage analyzer tidy

check:
	@fails=""; \
	for step in $(CHECK_STEPS); do \
	  $(MAKE) -s --no-print-directory $$step || fails="$$fails $$step"; \
	done; \
	if [ -z "$$fails" ]; then echo "TEST OVERALL: PASSED"; else echo "TEST OVERALL: FAILED at:$$fails"; fi; \
	[ -z "$$fails" ]

$(BINDIR) $(COVDIR) $(ANADIR):
	mkdir -p $@

$(GOLDEN_GCC): $(CORE_SRC) $(HARNESS) $(GOLDEN_MAIN) $(HARNESS_H) | $(BINDIR)
	$(CXX_GCC) $(CXXFLAGS) -o $@ $(GOLDEN_MAIN) $(HARNESS) $(CORE_SRC)

$(GOLDEN_CLANG): $(CORE_SRC) $(HARNESS) $(GOLDEN_MAIN) $(HARNESS_H) | $(BINDIR)
	$(CXX_CLANG) $(CXXFLAGS) -o $@ $(GOLDEN_MAIN) $(HARNESS) $(CORE_SRC)

$(TEST_GCC): $(CORE_SRC) $(HARNESS) $(TEST_MAIN) $(HARNESS_H) | $(BINDIR)
	$(CXX_GCC) $(CXXFLAGS) -o $@ $(TEST_MAIN) $(HARNESS) $(CORE_SRC)

$(TEST_CLANG): $(CORE_SRC) $(HARNESS) $(TEST_MAIN) $(HARNESS_H) | $(BINDIR)
	$(CXX_CLANG) $(CXXFLAGS) -o $@ $(TEST_MAIN) $(HARNESS) $(CORE_SRC)

$(SAN_GCC): $(CORE_SRC) $(HARNESS) $(TEST_MAIN) $(HARNESS_H) | $(BINDIR)
	$(CXX_GCC) $(CXXFLAGS) $(SANFLAGS) -o $@ $(TEST_MAIN) $(HARNESS) $(CORE_SRC)

$(SAN_CLANG): $(CORE_SRC) $(HARNESS) $(TEST_MAIN) $(HARNESS_H) | $(BINDIR)
	$(CXX_CLANG) $(CXXFLAGS) $(SANFLAGS) $(SAN_CLANG_EXTRA) -o $@ $(TEST_MAIN) $(HARNESS) $(CORE_SRC)

$(SAN_FULL): $(CORE_SRC) $(HARNESS) $(TEST_MAIN) $(HARNESS_H) | $(BINDIR)
	$(CXX_GCC) $(CXXFLAGS) $(SANFLAGS) $(UBSAN_FULL) -o $@ $(TEST_MAIN) $(HARNESS) $(CORE_SRC)

$(TEST_DBG): $(CORE_SRC) $(HARNESS) $(TEST_MAIN) $(HARNESS_H) | $(BINDIR)
	$(CXX_GCC) $(CXXFLAGS) $(DBGFLAGS) -o $@ $(TEST_MAIN) $(HARNESS) $(CORE_SRC)

golden-gcc: $(GOLDEN_GCC)
	$(GOLDEN_GCC) $(GOLDEN) $(PROFILE)

golden-clang: $(GOLDEN_CLANG)
	$(GOLDEN_CLANG) $(GOLDEN) $(PROFILE)

# Intentional regeneration of the committed golden reference.
update-golden: golden-gcc
	@echo "Golden reference updated intentionally: $(GOLDEN)"

test-gcc: $(TEST_GCC)
	$(TEST_GCC) $(GOLDEN) $(PROFILE)

test-clang: $(TEST_CLANG)
	$(TEST_CLANG) $(GOLDEN) $(PROFILE)

ifeq ($(ASAN_GCC),yes)
san-gcc: $(SAN_GCC)
	$(SAN_GCC) $(GOLDEN) $(PROFILE)
else
san-gcc:
	@echo "WARNING: san-gcc NOT RUN - the address/undefined sanitizer runtime (libasan/libubsan) is not installed for $(CXX_GCC)."
endif

ifeq ($(ASAN_CLANG),yes)
san-clang: $(SAN_CLANG)
	$(SAN_CLANG) $(GOLDEN) $(PROFILE)
else
san-clang:
	@echo "WARNING: san-clang NOT RUN - the address/undefined sanitizer runtime (compiler-rt) is not installed for $(CXX_CLANG)."
endif

ifeq ($(ASAN_GCC),yes)
san-full-gcc: $(SAN_FULL)
	$(SAN_FULL) $(GOLDEN) $(PROFILE)
else
san-full-gcc:
	@echo "WARNING: san-full-gcc NOT RUN - the address/undefined sanitizer runtime (libasan/libubsan) is not installed for $(CXX_GCC)."
endif

dbg-gcc: $(TEST_DBG)
	$(TEST_DBG) $(GOLDEN) $(PROFILE)

# Coverage: objects go to build/cov (.gcno/.gcda land there), binary to build/bin
$(COVDIR)/AirfoilData.o: ../src/AirfoilData.cpp | $(COVDIR)
	$(CXX_GCC) $(CXXFLAGS) $(COVFLAGS) -c $< -o $@

$(COVDIR)/AkimaInterpolation.o: ../src/AkimaInterpolation.cpp | $(COVDIR)
	$(CXX_GCC) $(CXXFLAGS) $(COVFLAGS) -c $< -o $@

$(COVDIR)/harness_common.o: $(HARNESS) $(HARNESS_H) | $(COVDIR)
	$(CXX_GCC) $(CXXFLAGS) $(COVFLAGS) -c $< -o $@

$(COVDIR)/test_airfoildata.o: $(TEST_MAIN) $(HARNESS_H) | $(COVDIR)
	$(CXX_GCC) $(CXXFLAGS) $(COVFLAGS) -c $< -o $@

$(TEST_COV): $(COV_OBJS) | $(BINDIR)
	$(CXX_GCC) $(CXXFLAGS) $(COVFLAGS) -o $@ $(COV_OBJS)

coverage: $(TEST_COV)
	$(TEST_COV) $(GOLDEN) $(PROFILE)
	@gcov -f -o $(COVDIR) $(COVDIR)/AirfoilData.gcno $(COVDIR)/AkimaInterpolation.gcno 2>/dev/null | c++filt > $(COVDIR)/gcov_func.txt; \
	awk '/^Function /{name=$$0; sub(/^Function /,"",name)} /^Lines executed:/{if(name ~ /AirfoilData::|AkimaInterpolation::/){tot++; if($$0 ~ /^Lines executed:0\.00%/) miss[name]=1}} END{m=0; for(k in miss)m++; printf "Method coverage: %d/%d methods tested\n", tot-m, tot; for(k in miss) print "  NOT tested: " k}' $(COVDIR)/gcov_func.txt

analyzer: | $(ANADIR)
	$(CXX_GCC) -std=c++17 -Wall -Wextra -I../src -fanalyzer -Wno-analyzer-use-of-uninitialized-value -c ../src/AirfoilData.cpp -o $(ANADIR)/analyzer_ad.o
	$(CXX_GCC) -std=c++17 -Wall -Wextra -I../src -fanalyzer -Wno-analyzer-use-of-uninitialized-value -c ../src/AkimaInterpolation.cpp -o $(ANADIR)/analyzer_akima.o

tidy:
	clang-tidy --checks='clang-analyzer-*' ../src/AirfoilData.cpp ../src/AkimaInterpolation.cpp -- -std=c++17 -I../src

memcheck: $(TEST_GCC)
	valgrind --leak-check=full --error-exitcode=1 $(TEST_GCC) $(GOLDEN) $(PROFILE)

sharp: san-full-gcc dbg-gcc coverage analyzer tidy memcheck

strict:
	$(MAKE) STRICT=-Werror test-gcc

clean:
	rm -rf build
	rm -f *.gcov *.gcno *.gcda
	rm -f harness_saved.dat harness_should_not_exist.dat  # GOLDEN (reference) is kept
