CXX=g++
NVCXX=nvcc
CXXFLAGS=-Wall -Werror -pedantic -std=c++17 -O3
NVCXXFLAGS=--expt-relaxed-constexpr -Xptxas -O3 -use_fast_math
BIN=icp

CPU_SRC=CPU/main.cc CPU/icp.cc CPU/matrices.cc
GPU_SRC=GPU/matrices.cu GPU/main.cu GPU/icp.cu
GPU_OPTI_SRC=GPU-opti/matrices.cu GPU-opti/main.cu GPU-opti/icp.cu

EIGEN_PATH=./eigen-3.3.8

all: gpu-opti

cpu:
	$(CXX) $(CXXFLAGS) -o $(BIN) -I $(EIGEN_PATH) $(CPU_SRC)

gpu:
	$(NVCXX) $(NVCXXFLAGS) -o $(BIN) $(GPU_SRC)

gpu-opti:
	$(NVCXX) $(NVCXXFLAGS) -o $(BIN) $(GPU_OPTI_SRC)

clean:
	rm CPU/*.o $(BIN)

.PHONY: all cpu gpu clean
