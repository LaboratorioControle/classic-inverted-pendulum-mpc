#ifndef MPC_H
#define MPC_H

#include <vector>
#include <Arduino.h>
#include <qpOASES.hpp>


struct Matrix {
    size_t r, c;
    std::vector<float> d;
    Matrix(): r(0), c(0) {}
    Matrix(size_t r_, size_t c_, float v=0.0): r(r_), c(c_), d(r_*c_, v) {}
    void resize(size_t r_, size_t c_, float v=0.0){ r=r_; c=c_; d.assign(r*c, v); }
    float& operator()(size_t i, size_t j){ return d[i*c + j]; }
    float  operator()(size_t i, size_t j) const { return d[i*c + j]; }
};

class MPC {
public:
    Matrix A, B, Cc, Dc, Cr;

    size_t n, nu, ny, nc;
    size_t N;

    Matrix Qu, Qy;

    Matrix ycmax, ycmin, deltamax, deltamin;
    Matrix umax, umin;

    Matrix H, F1, F2, F3;
    std::vector<qpOASES::real_t> H_qp, A_qp, lb_qp, ub_qp;
    Matrix Aineq, G1, G2, G3;

    std::vector<float> utildemax;
    std::vector<float> utildemin;

    MPC();

    void compute_MPC_Matrices();
    float compute_MPC_Command(float ulast, float pos_spt, float estados[4]);
    void printMatrix(const Matrix& M);
    

private:
    qpOASES::QProblem *qp = nullptr;
    bool qp_initialized = false;

    Matrix generate_yref(float pos_spt);
    std::vector<qpOASES::real_t> matrix_to_realt(const Matrix& M);
    std::vector<qpOASES::real_t> vector_to_realt(const std::vector<float>& v);
    void compute_Cost_Matrices();
    void compute_Constraints_Matrices();
};

#endif
