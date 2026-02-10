#ifndef MPC_H
#define MPC_H

#include <vector>
#include <Arduino.h>
#include <qpOASES.hpp>
#include <MessageHandling.hpp>

// Variáveis do MPC
#define N 35 // Horizonte de predição
#define n 4  // Número de estados
#define nc 3  // Número de estados com restrições
#define ny 2  // Número de estados regulados
#define nu 1  // Número de sinais de comandos
#define nre 7  // Número de pontos de parametrização/número de exponenciais

#define np (nu*nre) 
#define nU (N*nu)
#define nA (2*N*nc + 2*N*nu)
#define nAr (2*N*nc + 4*N*nu)

struct Matrix {
    size_t r, c;
    std::vector<float> d;
    Matrix(): r(0), c(0) {}
    Matrix(size_t r_, size_t c_, float v=0.0): r(r_), c(c_), d(r_*c_, v) {}
    void resize(size_t r_, size_t c_, float v=0.0){ r=r_; c=c_; d.assign(r*c, v); }
    float& operator()(size_t i, size_t j){ return d[i*c + j]; }
    float  operator()(size_t i, size_t j) const { return d[i*c + j]; }
};

enum class MPCForm {
    CLASSIC,
    LINEAR,
    EXPONENCIAL
};

class MPC {
public:
    Matrix A, B, Cc, Dc, Cr;

    Matrix Qu, Qy;

    Matrix ycmax, ycmin, deltamax, deltamin;
    Matrix umax, umin;

    MPC(MPCForm form = MPCForm::CLASSIC, int nWSR = 1000);

    void compute_MPC_Matrices();
    void compute_MPC_Matrices(float* pontos);
    void compute_MPC_Matrices(float* lambda, float alpha, float tau);
    float* compute_MPC_Command(float ulast, float* spt, float* err);

    
    private:
    qpOASES::QProblem *qp = nullptr;
    bool qp_initialized = false;
    MPCForm form_;
    
    //Matrizes calculadas offline
    qpOASES::real_t H[nU * nU];
    qpOASES::real_t F1[nU * n];
    qpOASES::real_t F2[nU * N*ny];
    qpOASES::real_t F3[nU * nu];
    
    qpOASES::real_t Aineq[nA * nU];
    qpOASES::real_t G1[nA * n];
    qpOASES::real_t G2[nA * nu];
    qpOASES::real_t G3[nA];
    
    //Matrizes de limite do sinal de comando
    qpOASES::real_t utildemax[nU];
    qpOASES::real_t utildemin[nU]; 
    
    // Matrizes calculadas online
    qpOASES::real_t yref[N * ny];
    qpOASES::real_t F[N];
    qpOASES::real_t Bineq[2*n*N];
    qpOASES::real_t qp_opt[nU];
    
    // Matrizes para a parametrização
    qpOASES::real_t H_p[np * np];
    qpOASES::real_t Aineq_p[nAr * np];
    qpOASES::real_t F_p[np];
    qpOASES::real_t Bineq_p[nAr];
    
    qpOASES::real_t u_[nu];
    qpOASES::real_t u_full[nU];
    int nWSR;
    
    
    // Matrizes de seleção para os casos parametrizados
    qpOASES::real_t Pi_r[nU * np];
    qpOASES::real_t Pi_e[nU * nre];

    void generate_yref(const float* spt, qpOASES::real_t* yref);
    void matrix_to_realt(const Matrix& M, qpOASES::real_t* result);
    void compute_Cost_Matrices();
    void compute_Constraints_Matrices();
    void build_cost_vector(float* err);
    void build_constraints(float* err, float ulast);
    void compute_util_opt();

    void init_solver_qp(int size_qp);
    void solver_qp();

    void compute_Bineq_reduced(qpOASES::real_t* Pi_ref);
    void compute_Aineq_reduced(qpOASES::real_t* Pi_ref);
    void compute_F_reduced(qpOASES::real_t* Pi_ref);
    void compute_H_reduced(qpOASES::real_t* Pi_ref);
    
    void compute_Pi_e(float* lambda, float alpha, float tau);
    void compute_Pi_r(float* pontos);
};

#endif
