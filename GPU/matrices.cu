#include "matrices.cuh"
#include <stdlib.h>
#include <assert.h>

#define checkCUDAError(val) { checkError((val), #val, __FILE__, __LINE__); }    // in-line regular function

void checkError(cudaError_t code, char const * func, const char *file, const int line)
{
    if (code != cudaSuccess) 
    {
        std::cerr << "CUDA error returned from \"" << func << "\" at "
                  << file << ":" << line << "\nError code: " << code
                  << "(" << cudaGetErrorString(code) << ")\n";
        cudaDeviceReset();
        exit(code);
    }
}

__global__ void mat_init(float* buffer, int height, int width, int value) {
    int i = blockDim.x * blockIdx.x + threadIdx.x;
    //int j = blockDim.y * blockIdx.y + threadIdx.y;
    if (i >= width * height) return;

    buffer[i] = value;
}

Mat::Mat(int height, int width)
    : m_height(height)
    , m_width(width)
    , m_buffer((float*) calloc(height * width, sizeof(float)))
{}

// Need to use a custom kernel instead of CudaMemSet because we operate of float pointers
Mat::Mat(int height, int width, float value)
    : m_height{height}
    , m_width{width}
{
    std::size_t buffer_size = height * width;
    this->m_buffer = (float*) malloc(height * width * sizeof(float));
    float* d_buffer;
    checkCUDAError(cudaMalloc(&d_buffer, height * width * sizeof(float)));

    cudaDeviceProp prop;
    cudaGetDeviceProperties(&prop, 0);
    std::size_t threadsPerBlock = (buffer_size < prop.maxThreadsPerBlock) ? buffer_size : prop.maxThreadsPerBlock;
    std::size_t nbBlocks = buffer_size / threadsPerBlock + 1;
    mat_init<<<nbBlocks, threadsPerBlock>>>(d_buffer, height, width, value);
    cudaDeviceSynchronize();
    checkCUDAError(cudaMemcpy(this->m_buffer, d_buffer, height * width * sizeof(float), cudaMemcpyDeviceToHost));
    cudaFree(d_buffer);
}

Mat::Mat(float* list_init, int height, int width)
    : m_height(height)
    , m_width(width)
{
    std::size_t buffer_size = height * width;
    this->m_buffer = (float*) malloc(buffer_size * sizeof(float));
    checkCUDAError(cudaMemcpy(this->m_buffer, list_init, buffer_size * sizeof(float), cudaMemcpyHostToHost));
}

Mat::Mat(float* list_init, int height)
    : Mat(list_init, height, 1)
    {}

Mat::Mat(const Mat& m)
    : Mat(m.m_buffer, m.m_height, m.m_width)
    {}

Mat Mat::copy() const
{ return Mat(m_buffer, m_height, m_width);}


void Mat::operator=(const Mat& other)
{
    m_height = other.m_height;
    m_width = other.m_width;
    checkCUDAError(cudaMemcpy(m_buffer, other.m_buffer,
                              m_height * m_width * sizeof(float), cudaMemcpyHostToHost));
}

Mat::~Mat(){
    free(this->m_buffer);
}

// I don't think that using a kernel (with all the overhead needed) will be faster than a little for loop
Mat Mat::eye(int dim)
{
    Mat ret(dim, dim);
    for (int i = 0; i < dim; ++i)
        ret.m_buffer[i * ret.m_width + i] = 1;
    return ret;
}

// Internet say, use a loop for k to avoid concurrency problem
__global__ void dot_kernel(float* self, float* other, float* ret,
                           int s_height, int s_width, int o_width){
    int th = blockDim.x * blockIdx.x + threadIdx.x;

    if (th >= s_height * o_width) return;

    int i = th / o_width; //0 to height
    int j = th % o_width; //0 to width
    for (int k = 0; k < s_width; ++k)
        ret[i * o_width + j] += self[i * s_width + k] * other[k * o_width + j];
}

Mat Mat::dot(const Mat& other)
{
    if (m_width != other.m_height)
    {
        printf("Invalid dot product, shapes do not match {%i, %i} vs {%i, %i}",
               m_height, m_width, other.m_height, other.m_width);
        throw "Invalid dot product";
    }

    Mat ret(m_height, other.m_width);
    float* ret_buffer;
    checkCUDAError(cudaMalloc(&ret_buffer, ret.m_height * ret.m_width* sizeof(float)));

    float* self_buffer;
    checkCUDAError(cudaMalloc(&self_buffer, m_height * m_width* sizeof(float)));
    checkCUDAError(cudaMemcpy(self_buffer, m_buffer, m_height * m_width * sizeof(float), cudaMemcpyHostToDevice));
    float* other_buffer;
    checkCUDAError(cudaMalloc(&other_buffer, other.m_height * other.m_width * sizeof(float)));
    checkCUDAError(cudaMemcpy(other_buffer, other.m_buffer,
                              other.m_height * other.m_width * sizeof(float), cudaMemcpyHostToDevice));

    cudaDeviceProp prop;
    cudaGetDeviceProperties(&prop, 0);
    std::size_t buffer_size = ret.m_height * ret.m_width;
    std::size_t threadsPerBlock = (buffer_size < prop.maxThreadsPerBlock)
        ? buffer_size : prop.maxThreadsPerBlock;
    std::size_t nbBlocks = buffer_size / threadsPerBlock + 1;
    dot_kernel<<<nbBlocks, threadsPerBlock>>>(self_buffer, other_buffer, ret_buffer,
                                              m_height, m_width, other.m_width);
    cudaDeviceSynchronize();

    checkCUDAError(cudaMemcpy(ret.m_buffer, ret_buffer, ret.m_height * ret.m_width * sizeof(float),
                              cudaMemcpyDeviceToHost));

    cudaFree(ret_buffer);
    cudaFree(self_buffer);
    cudaFree(other_buffer);
    return ret;
}

__global__ void T_kernel(float* self, float* ret, int s_height, int s_width) {
    int th = blockDim.x * blockIdx.x + threadIdx.x;

    if (th >= s_height * s_width) return;

    int i = th / s_width; //0 to height
    int j = th % s_width; //0 to width
    ret[j * s_height + i] = self[i * s_width + j];
}

Mat Mat::T() {
    Mat ret(m_width, m_height);
    float* ret_buffer;
    checkCUDAError(cudaMalloc(&ret_buffer, ret.m_height * ret.m_width * sizeof(float)));

    float* self_buffer;
    checkCUDAError(cudaMalloc(&self_buffer, m_height * m_width* sizeof(float)));
    checkCUDAError(cudaMemcpy(self_buffer, m_buffer, m_height * m_width * sizeof(float), cudaMemcpyHostToDevice));

    cudaDeviceProp prop;
    cudaGetDeviceProperties(&prop, 0);
    std::size_t buffer_size = ret.m_height * ret.m_width;
    std::size_t threadsPerBlock = (buffer_size < prop.maxThreadsPerBlock)
        ? buffer_size : prop.maxThreadsPerBlock;
    std::size_t nbBlocks = buffer_size / threadsPerBlock + 1;
    T_kernel<<<nbBlocks, threadsPerBlock>>>(self_buffer, ret_buffer, m_height, m_width);
    cudaDeviceSynchronize();

    checkCUDAError(cudaMemcpy(ret.m_buffer, ret_buffer, ret.m_height * ret.m_width * sizeof(float),
                              cudaMemcpyDeviceToHost));

    cudaFree(ret_buffer);
    cudaFree(self_buffer);
    return ret;
}

__global__ void add_kernel(float* self, float* other, float* ret, int s_height, int s_width) {
    int th = blockDim.x * blockIdx.x + threadIdx.x;

    if (th >= s_height * s_width) return;
    ret[th] = self[th] + other[th];
}

__global__ void add_broadcast_kernel(float* self, float* other, float* ret, int s_height, int s_width) {
    int th = blockDim.x * blockIdx.x + threadIdx.x;

    if (th >= s_height * s_width) return;
    int i = th / s_width; //0 to height
    int j = th % s_width; //0 to width
    ret[i * s_width + j] = self[i * s_width + j] + other[j];
}

Mat Mat::operator+(const Mat& other) const{
    if ((this->m_width != other.m_width) || (m_height != other.m_height && other.m_height != 1))
    {
        printf("Could not add matrices, dimensions do not match {%i, %i} vs {%i, %i}",
            this->m_height, this->m_width, other.m_height, other.m_width);
        throw "Invalid addition";
    }


    Mat ret(m_height, m_width);
    float* ret_buffer;
    checkCUDAError(cudaMalloc(&ret_buffer, ret.m_height * ret.m_width* sizeof(float)));

    float* self_buffer;
    checkCUDAError(cudaMalloc(&self_buffer, m_height * m_width* sizeof(float)));
    checkCUDAError(cudaMemcpy(self_buffer, m_buffer, m_height * m_width * sizeof(float), cudaMemcpyHostToDevice));
    float* other_buffer;
    checkCUDAError(cudaMalloc(&other_buffer, other.m_height * other.m_width * sizeof(float)));
    checkCUDAError(cudaMemcpy(other_buffer, other.m_buffer,
                              other.m_height * other.m_width * sizeof(float), cudaMemcpyHostToDevice));

    cudaDeviceProp prop;
    cudaGetDeviceProperties(&prop, 0);
    std::size_t buffer_size = ret.m_height * ret.m_width;
    std::size_t threadsPerBlock = (buffer_size < prop.maxThreadsPerBlock)
        ? buffer_size : prop.maxThreadsPerBlock;
    std::size_t nbBlocks = buffer_size / threadsPerBlock + 1;

    if (m_height == other.m_height)
        add_kernel<<<nbBlocks, threadsPerBlock>>>(self_buffer, other_buffer, ret_buffer,
                                                  m_height, m_width);
    else
        add_broadcast_kernel<<<nbBlocks, threadsPerBlock>>>(self_buffer, other_buffer, ret_buffer,
                                                  m_height, m_width);

    cudaDeviceSynchronize();

    checkCUDAError(cudaMemcpy(ret.m_buffer, ret_buffer, ret.m_height * ret.m_width * sizeof(float),
                              cudaMemcpyDeviceToHost));

    cudaFree(ret_buffer);
    cudaFree(self_buffer);
    cudaFree(other_buffer);
    return ret;
}

__global__ void sub_kernel(float* self, float* other, float* ret, int s_height, int s_width) {
    int th = blockDim.x * blockIdx.x + threadIdx.x;

    if (th >= s_height * s_width) return;
    ret[th] = self[th] - other[th];
}

__global__ void sub_broadcast_kernel(float* self, float* other, float* ret, int s_height, int s_width) {
    int th = blockDim.x * blockIdx.x + threadIdx.x;

    if (th >= s_height * s_width) return;
    int i = th / s_width; //0 to height
    int j = th % s_width; //0 to width
    ret[i * s_width + j] = self[i * s_width + j] - other[j];
}

Mat Mat::operator-(const Mat& other) const{
    if ((this->m_width != other.m_width) || (m_height != other.m_height && other.m_height != 1))
    {
        printf("Could not add matrices, dimensions do not match {%i, %i} vs {%i, %i}",
            this->m_height, this->m_width, other.m_height, other.m_width);
        throw "Invalid addition";
    }


    Mat ret(m_height, m_width);
    float* ret_buffer;
    checkCUDAError(cudaMalloc(&ret_buffer, ret.m_height * ret.m_width* sizeof(float)));

    float* self_buffer;
    checkCUDAError(cudaMalloc(&self_buffer, m_height * m_width* sizeof(float)));
    checkCUDAError(cudaMemcpy(self_buffer, m_buffer, m_height * m_width * sizeof(float), cudaMemcpyHostToDevice));
    float* other_buffer;
    checkCUDAError(cudaMalloc(&other_buffer, other.m_height * other.m_width * sizeof(float)));
    checkCUDAError(cudaMemcpy(other_buffer, other.m_buffer,
                              other.m_height * other.m_width * sizeof(float), cudaMemcpyHostToDevice));

    cudaDeviceProp prop;
    cudaGetDeviceProperties(&prop, 0);
    std::size_t buffer_size = ret.m_height * ret.m_width;
    std::size_t threadsPerBlock = (buffer_size < prop.maxThreadsPerBlock)
        ? buffer_size : prop.maxThreadsPerBlock;
    std::size_t nbBlocks = buffer_size / threadsPerBlock + 1;

    if (m_height == other.m_height)
        sub_kernel<<<nbBlocks, threadsPerBlock>>>(self_buffer, other_buffer, ret_buffer,
                                                  m_height, m_width);
    else
        sub_broadcast_kernel<<<nbBlocks, threadsPerBlock>>>(self_buffer, other_buffer, ret_buffer,
                                                  m_height, m_width);

    cudaDeviceSynchronize();

    checkCUDAError(cudaMemcpy(ret.m_buffer, ret_buffer, ret.m_height * ret.m_width * sizeof(float),
                              cudaMemcpyDeviceToHost));

    cudaFree(ret_buffer);
    cudaFree(self_buffer);
    cudaFree(other_buffer);
    return ret;
}

__global__ void normalize_kernel(float *A, float *I, int n, int x, bool diag){
    int th = blockDim.x * blockIdx.x + threadIdx.x;

    if (th >= n * n) return;
    int i = th / n; //0 to height
    int j = th % n; //0 to width
    if ((!diag && (i == x && i != j)) || (diag && (i == x && i == j))){
        I[i * n + j] /= A[x * n + x];
        A[i * n + j] /= A[x * n + x];
    }
}

__global__ void gaussjordan_kernel(float *A, float *I, int n, int x)
{
    int th = blockDim.x * blockIdx.x + threadIdx.x;

    if (th >= n * n) return;
    int i = th / n; //0 to height
    int j = th % n; //0 to width

    if (i != x) {
        I[i * n + j] -= I[x * n + j] * A[i * n + x];
        if (j != x){
            A[i * n + j] -= A[x * n + j] * A[i * n + x];
        }
    }
}

__global__ void zero_kernel(float *A, int n, int x){
    int th = blockDim.x * blockIdx.x + threadIdx.x;

    if (th >= n * n) return;
    int i = th / n; //0 to height
    int j = th % n; //0 to width

    if (i != x && j == x){
        A[i * n + j] = 0;
    }
}

Mat Mat::inverse() const
{
    Mat ret = eye(m_height);
    float* ret_buffer;
    checkCUDAError(cudaMalloc(&ret_buffer, ret.m_height * ret.m_width * sizeof(float)));
    checkCUDAError(cudaMemcpy(ret_buffer, ret.m_buffer, ret.m_height * ret.m_width * sizeof(float),
                              cudaMemcpyHostToDevice));

    float* self_buffer;
    checkCUDAError(cudaMalloc(&self_buffer, m_height * m_width * sizeof(float)));
    checkCUDAError(cudaMemcpy(self_buffer, m_buffer, m_height * m_width * sizeof(float), cudaMemcpyHostToDevice));

    cudaDeviceProp prop;
    cudaGetDeviceProperties(&prop, 0);
    std::size_t buffer_size = ret.m_height * ret.m_width;
    std::size_t threadsPerBlock = (buffer_size < prop.maxThreadsPerBlock)
        ? buffer_size : prop.maxThreadsPerBlock;
    std::size_t nbBlocks = buffer_size / threadsPerBlock + 1;

    for (int i = 0; i < m_height; ++i)
    {
        normalize_kernel<<<nbBlocks, threadsPerBlock>>>(self_buffer, ret_buffer,
                                                        m_height, i, 0);
        normalize_kernel<<<nbBlocks, threadsPerBlock>>>(self_buffer, ret_buffer,
                                                        m_height, i, 1);
        gaussjordan_kernel<<<nbBlocks, threadsPerBlock>>>(self_buffer, ret_buffer,
                                                        m_height, i);
        zero_kernel<<<nbBlocks, threadsPerBlock>>>(self_buffer, m_height, i);
    }

    checkCUDAError(cudaMemcpy(ret.m_buffer, ret_buffer, ret.m_height * ret.m_width * sizeof(float),
                              cudaMemcpyDeviceToHost));

    cudaFree(self_buffer);
    cudaFree(ret_buffer);

    return ret;
}

void Mat::print() const {
    std::cout << "{\n";
    for (int i = 0; i < this->m_height; ++i) {
        std::cout << "  { ";
        for (int j = 0; j < this->m_width;) {
            std::cout << this->m_buffer[i * this->m_width + j];
            if (++j < this->m_width)
                std::cout << ", ";
        }
        std::cout << " }\n";
    }
    std::cout << "}\n";
}