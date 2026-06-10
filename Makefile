# Makefile for FisheyeDewarp
# ──────────────────────────────────────────────────────────────────────────────
# Targets:
#   make             → build host binary (x86-64, debug-friendly O2)
#   make release     → build host binary with -O3 -DNDEBUG
#   make aarch64     → cross-compile for AArch64 with NEON
#   make clean       → remove build artefacts

CXX        := g++
CXXFLAGS   := -std=c++17 -Wall -Wextra -O2
LDFLAGS    := -lm

SRCS       := main.cpp fisheye_dewarp.cpp
TARGET     := dewarp_test

.PHONY: all release aarch64 clean

all: $(TARGET)

$(TARGET): $(SRCS) fisheye_dewarp.hpp
	$(CXX) $(CXXFLAGS) $(SRCS) -o $@ $(LDFLAGS)
	@echo "Built: $(TARGET)"

release: CXXFLAGS := -std=c++17 -Wall -O3 -DNDEBUG -ffast-math
release: $(TARGET)

aarch64:
	aarch64-linux-gnu-g++ \
	    -std=c++17 -O3 -DNDEBUG \
	    -march=armv8-a+simd -mfpu=neon-fp-armv8 \
	    -funsafe-math-optimizations \
	    $(SRCS) -o dewarp_test_aarch64 -lm
	@echo "Built: dewarp_test_aarch64"

clean:
	rm -f $(TARGET) dewarp_test_aarch64 *.pgm
