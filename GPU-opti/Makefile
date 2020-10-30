CXXFLAGS=--expt-relaxed-constexpr -Xptxas -O3 -use_fast_math

SRC=matrices.cu main.cu icp.cu

all:
	nvcc $(SRC) $(CXXFLAGS) -o main

clean:
	rm $(OBJS) icp

