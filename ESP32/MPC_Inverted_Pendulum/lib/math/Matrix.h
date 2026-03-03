#include <vector>
#include <Arduino.h>

class Matrix {
public:
    size_t r, c;
    std::vector<float> d;
    Matrix(): r(0), c(0) {}
    Matrix(size_t r_, size_t c_, float v=0.0): r(r_), c(c_), d(r_*c_, v) {}
    void resize(size_t r_, size_t c_, float v=0.0){ 
        r=r_; 
        c=c_; 
        d.assign(r*c, v); 
    }

    float& operator()(size_t i, size_t j){ 
        return d[i*c + j]; 
    }

    float operator()(size_t i, size_t j) const { 
        return d[i*c + j]; 
    }

    static Matrix eye(int n_);
    static Matrix zeros(int r, int c);
    static Matrix transpose(const Matrix& A);
    static Matrix mul(const Matrix& A, const Matrix& B);
    static Matrix smul(float s, const Matrix& A);
    static void insertBlock(Matrix& dst, int r0, int c0, const Matrix& src);
    static Matrix P_i(int i, int n1, int N_);

};

Matrix operator+(const Matrix& A, const Matrix& B);
Matrix operator-(const Matrix& A, const Matrix& B);