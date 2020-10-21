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

Mat Mat::operator+(const Mat& other){
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

Mat Mat::operator-(const Mat& other){
    if (this->m_height != other.m_height || this->m_width != other.m_width)
    {
        printf("Could not subtract matrices, dimensions do not match {%i, %i} vs {%i, %i}",
            this->m_height, this->m_width, other.m_height, other.m_width);
        throw "Invalid subtraction";
    }

    Mat ret = Mat(this->m_height, this->m_width);
    for (int i = 0; i < this->m_height; i++) {
        for (int j = 0; j < this->m_width; j++) {
            ret.m_buffer[i][j] = this->m_buffer[i][j] - other.m_buffer[i][j];
        }
    }

    return ret;
}

Mat Mat::operator*(const float& factor){
    Mat ret = Mat(this->m_height, this->m_width);

    for (int i = 0; i < this->m_height; i++) {
        for (int j = 0; j < this->m_width; j++) {
            ret.m_buffer[i][j] = factor * this->m_buffer[i][j];
        }
    }

    return ret;
}

Mat Mat::operator*(const Mat& other){
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