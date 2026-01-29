#ifndef MPC_H
#define MPC_H

#include <vector>
#include <Arduino.h>
#include <qpOASES.hpp>

// Variáveis do MPC
#define N 35 // Horizonte de predição
#define n 4  // Número de estados
#define nc 3  // Número de estados com restrições
#define ny 2  // Número de estados regulados
#define nu 1  // Número de sinais de comandos


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

    Matrix Qu, Qy;

    Matrix ycmax, ycmin, deltamax, deltamin;
    Matrix umax, umin;

    //Matrizes calculadas offline
    qpOASES::real_t H[N*nu * N*nu];
    qpOASES::real_t F1[N*nu * n];
    qpOASES::real_t F2[N*nu * N*ny];
    qpOASES::real_t F3[N*nu * nu];

    qpOASES::real_t Aineq[(2*N*nc + 2*N*nu) * N*nu];
    qpOASES::real_t G1[(2*N*nc + 2*N*nu) * n];
    qpOASES::real_t G2[(2*N*nc + 2*N*nu) * nu];
    qpOASES::real_t G3[2*N*nc + 2*N*nu];

    //Matrizes de limite do sinal de comando
    qpOASES::real_t utildemax[N * nu];
    qpOASES::real_t utildemin[N * nu]; 
    
    // Matrizes calculadas online
    qpOASES::real_t yref[N * ny];
    qpOASES::real_t F[N];
    qpOASES::real_t Bineq[2*n*N];
    qpOASES::real_t utilde_opt[N * nu];

    MPC();

    void compute_MPC_Matrices();
    float compute_MPC_Command(float ulast, float* spt, float* err);
    void printMatrix(const Matrix& M);
    

private:
    qpOASES::QProblem *qp = nullptr;
    bool qp_initialized = false;

    void generate_yref(const float* spt, qpOASES::real_t* yref);
    void matrix_to_realt(const Matrix& M, qpOASES::real_t* result);
    void compute_Cost_Matrices();
    void compute_Constraints_Matrices();
};

#endif
