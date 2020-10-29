CXXFLAGS=-Wall -Werror -pedantic --expt-relaxed-constexpr -g

SRC=matrices.cu main.cu icp.cu

EIGEN_PATH=../eigen-3.3.8

all:
	nvcc $(SRC) -I $(EIGEN_PATH) -o main

clean:
	rm $(OBJS) icp

