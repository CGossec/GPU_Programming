#include "matrices.hh"

Mat::Mat(int height, int width)
    : m_height{height}
    , m_width{width}
{
    for (int i = 0; i < height; ++i) {
        std::vector<float> tmp;
        for (int j = 0; j < width; ++j) {
            tmp.push_back(0);
        }
        m_buffer.push_back(tmp);
    }
}

Mat::Mat(int height, int width, float value)
    : m_height{height}
    , m_width{width}
{
    for (int i = 0; i < height; ++i) {
        std::vector<float> tmp;
        for (int j = 0; j < width; ++j) {
            tmp.push_back(value);
        }
        m_buffer.push_back(tmp);
    }
}

Mat::Mat(std::vector<std::vector<float>>&& list_init){
    auto height = list_init.size();
    auto width = list_init[0].size();

    for (std::size_t i = 0; i < height; ++i)
        if (list_init[i].size() != width)
            throw "Invalid list initialization, internal vectors were not of same width.";

    m_height = height;
    m_width = width;
    m_buffer = list_init;
}

Mat::Mat(const Mat& m)
    : m_height(m.m_height)
    , m_width(m.m_width)
{
    for (int i = 0; i < m_height; ++i) {
        std::vector<float> tmp;
        for (int j = 0; j < m_width; ++j) {
            tmp.push_back(m[i][j]);
        }
        m_buffer.push_back(tmp);
    }
}

Mat Mat::eye(int dim)
{
    Mat ret(dim, dim);
    for (int i = 0; i < dim; ++i)
        ret[i][i] = 1;
    return ret;
}

Mat Mat::dot(const Mat& other){
    if (this->m_width != other.m_height)
    {
        printf("Invalid dot product, shapes do not match {%i, %i} vs {%i, %i}",
            this->m_height, this->m_width, other.m_height, other.m_width);
        throw "Invalid dot product";
    }

    Mat ret = Mat(this->m_height, other.m_width);

    for (int i = 0; i < this->m_height; ++i) {
        for (int j = 0; j < other.m_width; ++j){
            for (int k = 0; k < other.m_height; ++k) {
                ret.m_buffer[i][j] += this->m_buffer[i][k] * other.m_buffer[k][j];
            }
        }
    }
    return ret;
}

Mat Mat::dot(const std::vector<float>& other)
{
    if (this->m_width != other.size())
    {
        printf("Invalid dot product, shapes do not match {%i, %i} vs {%i, 1}",
               this->m_height, this->m_width, other.size());
        throw "Invalid dot product";
    }
    Mat vector({other});
    return dot(vector);
}

Mat Mat::T() {
    auto ret = Mat(this->m_width, this->m_height);
    for (int i = 0; i < this->m_height; i++)
        for (int j = 0; j < this->m_width; j++)
            ret.m_buffer[j][i] = this->m_buffer[i][j];
    return ret;
}

std::vector<float> Mat::operator[](const int pos) const {
    return this->m_buffer[pos];
}

std::vector<float>& Mat::operator[](const int pos) {
    return this->m_buffer[pos];
}

Mat Mat::operator+(const Mat& other) const{
    if (this->m_height != other.m_height || this->m_width != other.m_width)
    {
        printf("Could not add matrices, dimensions do not match {%i, %i} vs {%i, %i}",
            this->m_height, this->m_width, other.m_height, other.m_width);
        throw "Invalid addition";
    }

    Mat ret = Mat(this->m_height, this->m_width);
    for (int i = 0; i < this->m_height; i++) {
        for (int j = 0; j < this->m_width; j++) {
            ret.m_buffer[i][j] = this->m_buffer[i][j] + other.m_buffer[i][j];
        }
    }

    return ret;
}

Mat Mat::operator-(const Mat& other) const{
    if ((this->m_width != other.m_width) || (m_height != other.m_height && other.m_height != 1))
    {
        printf("Could not subtract matrices, dimensions do not match {%i, %i} vs {%i, %i}",
            this->m_height, this->m_width, other.m_height, other.m_width);
        throw "Invalid subtraction";
    }

    Mat ret = Mat(this->m_height, this->m_width);
    if (m_height == other.m_height)
    {
        for (int i = 0; i < this->m_height; i++) {
            for (int j = 0; j < this->m_width; j++) {
                ret.m_buffer[i][j] = this->m_buffer[i][j] - other.m_buffer[i][j];
            }
        }
    }
    else
    {
        for (int i = 0; i < this->m_height; i++) {
            for (int j = 0; j < this->m_width; j++) {
                ret.m_buffer[i][j] = this->m_buffer[i][j] - other.m_buffer[0][j];
            }
        }
    }

    return ret;
}

Mat Mat::operator*(const float& factor) const{
    Mat ret = Mat(this->m_height, this->m_width);

    for (int i = 0; i < this->m_height; i++) {
        for (int j = 0; j < this->m_width; j++) {
            ret.m_buffer[i][j] = factor * this->m_buffer[i][j];
        }
    }

    return ret;
}

Mat Mat::operator*(const Mat& other) const{
    /**
     * Trying to replicate numpy broadcasting 
     * https://numpy.org/doc/stable/user/basics.broadcasting.html
     * TODO
     */
    //auto ret_h = max(this->m_height, other.m_height);
    //auto ret_w = max(this->m_width, other.m_width);
    //Mat ret = Mat(ret_h, ret_w);

    //for (int i = 0; i < this->m_height; ++i) {
    //    for (int j = 0; j < this->m_width; ++j){
    //            ret.m_buffer[i][j] += this->m_buffer[i][k] * other.m_buffer[k][j];
    //        }
    //    }
    //}
    //return ret;
    return Mat(1,1,-1);
}

void Mat::print() const {
    std::cout << "{\n";
    for (int i = 0; i < this->m_height; ++i) {
        std::cout << "  { ";
        for (int j = 0; j < this->m_width;) {
            std::cout << this->m_buffer[i][j];
            if (++j < this->m_width)
                std::cout << ", ";
        }
        std::cout << " }\n";
    }
    std::cout << "}\n";
}

Mat Mat::copy() const {
    Mat res = Mat(this->m_height, this->m_width);
    for (int h = 0; h < this->m_height; ++h)
        for (int w = 0; w < this->m_width; ++w)
            res.m_buffer[h][w] = this->m_buffer[h][w];
    return res;
}

float mean_vector(const std::vector<float>& v)
{
    float r = 0.;
    for (int i = 0; i < v.size(); ++i)
        r += v[i];
    return r / v.size();
}

Mat Mat::mean() const {
    std::vector<float> aggreagate(m_width, 0);
    for (int i = 0; i < m_height; ++i)
        for (int j = 0; j < m_width; ++j)
            aggreagate[j] += m_buffer[i][j];
    for (int i = 0; i < m_width; ++i)
        aggreagate[i] /= m_height;
    return Mat({aggreagate});
}

std::vector<std::tuple<float, Eigen::VectorXf>> get_eigen(Eigen::MatrixXf m) {
    Eigen::EigenSolver<Eigen::MatrixXf> eigensolver;

    eigensolver.compute(m);

    Eigen::VectorXf eigen_values = eigensolver.eigenvalues().real();
    Eigen::MatrixXf eigen_vectors = eigensolver.eigenvectors().real();
    std::vector<std::tuple<float, Eigen::VectorXf>> eigen_vectors_and_values;

    for(int i = 0; i < eigen_values.size(); i++){
        std::tuple<float, Eigen::VectorXf> vec_and_val(eigen_values[i], eigen_vectors.row(i));
        eigen_vectors_and_values.push_back(vec_and_val);
    }

    std::sort(eigen_vectors_and_values.begin(), eigen_vectors_and_values.end(),
              [&](const std::tuple<float, Eigen::VectorXf>& a, const std::tuple<float, Eigen::VectorXf>& b) -> bool{
                  return std::get<0>(a) <= std::get<0>(b);
              });

    return eigen_vectors_and_values;
}

std::vector<std::tuple<float, std::vector<float>>> Mat::eigen() const {
    Eigen::MatrixXf eigen_mat(m_height, m_width);

    for (int i = 0; i < m_height; ++i)
        for (int j = 0; j < m_width; ++j)
            eigen_mat(i, j) = m_buffer[i][j];

    auto eigen_value_vector = get_eigen(eigen_mat);

    std::vector<std::tuple<float, std::vector<float>>> ret;
    for (int i = 0; i < eigen_value_vector.size(); ++i)
    {
        std::vector<float> tmp;
        auto eigen_vector = std::get<1>(eigen_value_vector[i]);
        for (int j = 0; j < m_height; ++j)
            tmp.push_back(eigen_vector[j]);
        std::tuple<float, std::vector<float>> tup = std::make_tuple(std::get<0>(eigen_value_vector[i]), tmp);
        ret.push_back(tup);
    }

    return ret;
}
