CXX?=g++
CPPFLAGS=-Wall -Werror -pedantic -std=c++17 -O3

SRCS=main.cc icp.cc matrices.cc
OBJS=$(subst .cc,.o,$(SRCS))

all: main

main: $(OBJS)
	$(CXX) -o icp $(OBJS)

clean:
	rm $(OBJS) icp

