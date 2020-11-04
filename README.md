# GPGPU

AUTHORS : antoine.hacquard, celian.gossec, nicolas.ceccarello, nathan.coupermant

For this project, we were tasked with implementing an ICP (Iterative Closest Point) algorithm in CUDA.
The project was split in different steps:
 - First, work on getting a valid CPU implementation
 - Benchmark the CPU implementation to have a baseline
 - Work on getting a (better) GPU implementation
 - Benchmark the GPU implementation
 - Repeat steps 3 and 4, and add optimizations
Libraries were authorized but not encouraged, as few libraries work in CUDA.Using a library meant going from CPU to GPU would be more complicated.

## Installation

1. Get the eigen library with : `wget https://gitlab.com/libeigen/eigen/-/archive/3.3.8/eigen-3.3.8.zip`
2. Unzip it at the directory root : `unzip eigen-3.3.8.zip`
3. Compile the code and create a binary in the relevant directory:
```
make -j4 cpu # compile the CPU version of the ICP in the CPU directory
make -j4 gpu # compile the non optimized GPU version in the GPU directory
make -j4 gpu-opti # Compile the optimize GPU version in the GPU-opti directory
```

## Run

./{directory-where-you-create-the-binary}/icp datatest/{cloud-points}_src datatest/{cloud-points}_tgt
