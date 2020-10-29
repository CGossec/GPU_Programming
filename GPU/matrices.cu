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

Mat::Mat(float* list_init, int width)
    : Mat(list_init, 1, width)
    {}

Mat::Mat(const Mat& m)
    : Mat(m.m_buffer, m.m_height, m.m_width)
    {}

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

// __global__ void Mat::operator-(const Mat& other, Mat* ret) const{
//     if ((this->m_width != other.m_width) || (m_height != other.m_height && other.m_height != 1))
//     {
//         printf("Could not subtract matrices, dimensions do not match {%i, %i} vs {%i, %i}",
//             this->m_height, this->m_width, other.m_height, other.m_width);
//         throw "Invalid subtraction";
//     }

//     int i = blockDim.x*blockId.x + threadId.x;
//     int j = blockDim.y*blockId.y + threadId.y;
    
//     if (i == 0 && j == 0) {
//         // Initialize the return matrix
//         ret = Mat(this->m_height, this->m_width);
//     } 
//     else if (i >= this->m_height || j >= this->m_width) return;
    
//     if (m_height == other.m_height) {
//         ret->m_buffer[i][j] = this->m_buffer[i][j] - other.m_buffer[i][j];
//     }
//     else {
//         ret->m_buffer[i][j] = this->m_buffer[i][j] - other.m_buffer[0][j];
//     }
// }

// __global__ void Mat::operator*(const float& factor, Mat* ret) const{
//     int i = blockDim.x*blockId.x + threadId.x;
//     int j = blockDim.y*blockId.y + threadId.y;

//     if (i == 0 && j == 0) {
//         // Initialize the return matrix
//         ret = Mat(this->m_height, this->m_width);
//     } 
//     else if (i >= this->m_height || j >= this->m_width) return;

//     ret.m_buffer[i][j] = factor * this->m_buffer[i][j];
// }

// __global__ void Mat::operator*(const Mat& other, Mat* ret) const{
//     /**
//      * Trying to replicate numpy broadcasting 
//      * https://numpy.org/doc/stable/user/basics.broadcasting.html
//      * TODO
//      */
//     int ret_h = this->m_height;
//     if (other.m_height > ret_h) {
//         ret_h = other.m_height;
//     }
//     int ret_w = this->m_height;
//     if (other.m_height > ret_w) {
//         ret_w = other.m_width;
//     }
    
//     int i = blockDim.x*blockId.x + threadId.x;
//     int j = blockDim.y*blockId.y + threadId.y;

//     if (i == 0 && j == 0) {
//         // Initialize the return matrix
//         ret = Mat(ret_h, ret_w);
//     } else if (i >= ret_h || j >= ret_w) return;


//     if (this->m_height == 1 && other.m_width == 1){
//         ret->m_buffer[i][j] += this->m_buffer[0][i] * other.m_buffer[j][0];
//     }
//     else if (this->m_width == 1 && other.m_height == 1){
//         ret.m_buffer[i][j] += other.m_buffer[0][i] * this->m_buffer[j][0];
//     }
//     else{
//         printf("Could not broadcast matrices, dimensions do not match {%i, %i} vs {%i, %i}",
//                this->m_height, this->m_width, other.m_height, other.m_width);
//         assert(0); // Need cleaner asset
//     }
// }

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

// __global__ void Mat::copy(Mat* ret) const {
//     if (ret == NULL) {
//         // Initialize the return matrix
//         ret = Mat(this->m_height, this->m_width);
//     }

//     int i = blockDim.x*blockId.x + threadId.x;
//     int j = blockDim.y*blockId.y + threadId.y;
//     if (i >= this->m_height || j >= this->m_width) return;    
//     ret->m_buffer[h][w] = this->m_buffer[h][w];
// }

// float mean_vector(const std::vector<float>& v)
// {
//     float r = 0.;
//     for (std::size_t i = 0; i < v.size(); ++i)
//         r += v[i];
//     return r / v.size();
// }

// Mat Mat::mean() const {
//     std::vector<float> aggreagate(m_width, 0);
//     for (int i = 0; i < m_height; ++i)
//         for (int j = 0; j < m_width; ++j)
//             aggreagate[j] += m_buffer[i][j];
//     for (int i = 0; i < m_width; ++i)
//         aggreagate[i] /= m_height;
//     return Mat(std::vector<std::vector<float>>{aggreagate});
// }

// std::vector<std::tuple<float, Eigen::VectorXf>> get_eigen(Eigen::MatrixXf m) {
//     Eigen::EigenSolver<Eigen::MatrixXf> eigensolver;

//     eigensolver.compute(m);

//     Eigen::VectorXf eigen_values = eigensolver.eigenvalues().real();
//     Eigen::MatrixXf eigen_vectors = eigensolver.eigenvectors().real();
//     std::vector<std::tuple<float, Eigen::VectorXf>> eigen_vectors_and_values;

//     for(int i = 0; i < eigen_values.size(); i++){
//         std::tuple<float, Eigen::VectorXf> vec_and_val(eigen_values[i], eigen_vectors.row(i));
//         eigen_vectors_and_values.push_back(vec_and_val);
//     }

//     std::sort(eigen_vectors_and_values.begin(), eigen_vectors_and_values.end(),
//               [&](const std::tuple<float, Eigen::VectorXf>& a, const std::tuple<float, Eigen::VectorXf>& b) -> bool{
//                   return std::get<0>(a) <= std::get<0>(b);
//               });

//     return eigen_vectors_and_values;
// }

// std::vector<std::tuple<float, std::vector<float>>> Mat::eigen() const {
//     Eigen::MatrixXf eigen_mat(m_height, m_width);

//     for (int i = 0; i < m_height; ++i)
//         for (int j = 0; j < m_width; ++j)
//             eigen_mat(i, j) = m_buffer[i][j];

//     auto eigen_value_vector = get_eigen(eigen_mat);

//     std::vector<std::tuple<float, std::vector<float>>> ret;
//     for (std::size_t i = 0; i < eigen_value_vector.size(); ++i)
//     {
//         std::vector<float> tmp;
//         auto eigen_vector = std::get<1>(eigen_value_vector[i]);
//         for (int j = 0; j < m_height; ++j)
//             tmp.push_back(eigen_vector[j]);
//         std::tuple<float, std::vector<float>> tup = std::make_tuple(std::get<0>(eigen_value_vector[i]), tmp);
//         ret.push_back(tup);
//     }

//     return ret;
// }

// Mat Mat::inverse() const {
//     Eigen::MatrixXf eigen_mat(m_height, m_width);

//     for (int i = 0; i < m_height; ++i)
//         for (int j = 0; j < m_width; ++j)
//             eigen_mat(i, j) = m_buffer[i][j];


//     auto eigen_inverse = eigen_mat.inverse();

//     Mat ret(m_height, m_width);
//     for (int i = 0; i < m_height; ++i) {
//         for (int j = 0; j < m_width; ++j) {
//             ret[i][j] = eigen_inverse(i, j);
//         }
//     }

//     return ret;
// }
