CXX?=g++
CXXFLAGS=-Wall -Werror -pedantic -std=c++17 -O3

SRCS_LSTSQ=main.cc icp.cc matrices.cc
OBJS_LSTSQ=$(subst .cc,.o,$(SRCS_LSTSQ))

SRCS_MATLAB=main.cc icp-matlab.cc matrices.cc
OBJS_MATLAB=$(subst .cc,.o,$(SRCS_MATLAB))

EIGEN_PATH=./eigen-3.3.8

all: lstsq

lstsq:
	$(CXX) $(CXXFLAGS) -o icp -I $(EIGEN_PATH) $(SRCS_LSTSQ)

matlab:
	$(CXX) $(CXXFLAGS) -o icp -I $(EIGEN_PATH) $(SRCS_MATLAB)

clean:
	rm $(OBJS) icp

