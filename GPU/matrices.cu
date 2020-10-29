#include "matrices.cuh"
#include <assert.h>

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
    float* d_buffer = NULL;
    cudaMalloc((void **)&d_buffer, height * width * sizeof(float));

    cudaDeviceProp prop;
    cudaGetDeviceProperties(&prop, 0);
    std::size_t threadsPerBlock = (buffer_size < prop.maxThreadsPerBlock) ? buffer_size : prop.maxThreadsPerBlock;

    std::size_t nbBlocks = buffer_size / threadsPerBlock + 1;
    mat_init<<<nbBlocks, threadsPerBlock>>>(d_buffer, height, width, value);
    cudaDeviceSynchronize();
    cudaMemcpy(this->m_buffer, d_buffer, height * width * sizeof(float), cudaMemcpyDeviceToHost);
    cudaFree(d_buffer);
}

Mat::Mat(float* list_init, int height, int width)
    : m_height(height)
    , m_width(width)
{
    std::size_t buffer_size = height * width;
    this->m_buffer = (float*) malloc(buffer_size * sizeof(float));
    cudaMemcpy(this->m_buffer, list_init, buffer_size * sizeof(float), cudaMemcpyHostToHost);
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

// Mat Mat::eye(int dim)
// {
//     Mat ret(dim, dim);
//     for (int i = 0; i < dim; ++i)
//         ret[i][i] = 1;
//     return ret;
// }

// __global__ void Mat::dot(const Mat& other, Mat* ret){
//     if (this->m_width != other.m_height)
//     {
//         printf("Invalid dot product, shapes do not match {%i, %i} vs {%i, %i}",
//             this->m_height, this->m_width, other.m_height, other.m_width);
//         assert(this->m_width == other.m_height);
//     }
//     int i = blockDim.x*blockId.x + threadId.x;
//     int j = blockDim.y*blockId.y + threadId.y;
//     int k = blockDim.z*blockId.z + threadId.z;

//     if (i == 0 && j == 0 && k == 0) {
//         // Initialize the return matrix
//         ret = &Mat(this->m_height, other.m_width);
//     }

//     if (i >= this->m_height || j >= this->m_width || k >= this->m_height) return;
//     ret->m_buffer[i][j] += this->m_buffer[i][k] * other.m_buffer[k][j];
// }

// void Mat::dot(const float* other, int height, Mat* ret)
// {
//     if ((std::size_t)this->m_width != other.size())
//     {
//         printf("Invalid dot product, shapes do not match {%i, %i} vs {%zd, 1}",
//                this->m_height, this->m_width, other.size());
//         throw "Invalid dot product";
//     }
//     Mat vector(other, height);
//     dot(vector, ret); // Fix dot call
// }

// __global__ void Mat::T(Mat* ret) {
//     if (ret == NULL) {
//         // Initialize the return matrix
//         ret = &Mat(this->m_width, this->m_height);
//     }

//     int i = blockDim.x*blockId.x + threadId.x;
//     int j = blockDim.y*blockId.y + threadId.y;
//     if (i >= this->m_height || j >= this->m_width) return;
//     ret->m_buffer[j][i] = this->m_buffer[i][j];
// }

// float* Mat::operator[](const int pos) const {
//     return this->m_buffer[pos];
// }

// const float* Mat::operator[](const int pos) {
//     return this->m_buffer[pos];
// }

// __global__ void Mat::operator+(const Mat& other, Mat* ret) const{
//     if ((this->m_width != other.m_width) || (m_height != other.m_height && other.m_height != 1))
//     {
//         printf("Could not add matrices, dimensions do not match {%i, %i} vs {%i, %i}",
//             this->m_height, this->m_width, other.m_height, other.m_width);
//         throw "Invalid addition";
//     }

//     if (ret == NULL) {
//         // Initialize the return matrix
//         ret = Mat(this->m_height, this->m_width);
//     }

//     if (m_height == other.m_height)
//     {
//         int i = blockDim.x*blockId.x + threadId.x;
//         int j = blockDim.y*blockId.y + threadId.y;
//         if (i >= this->m_height || j >= this->m_width) return;    
//         ret->m_buffer[i][j] = this->m_buffer[i][j] + other.m_buffer[i][j];
//     }
//     else
//     {
//         int i = blockDim.x*blockId.x + threadId.x;
//         int j = blockDim.y*blockId.y + threadId.y;
//         if (i >= this->m_height || j >= this->m_width) return;    
//         ret->m_buffer[i][j] = this->m_buffer[i][j] + other.m_buffer[0][j];
//     }
// }

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
