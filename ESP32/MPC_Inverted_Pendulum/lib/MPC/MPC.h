#ifndef MPC_H
#define MPC_H

#include <qpOASES.hpp>
#include <Matrix.h>

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

// Lista de opções de MPC disponíveis (Solução clássica, parametrização trivial e exponencial)
enum class MPCForm {
    CLASSIC,
    LINEAR,
    EXPONENCIAL
};

class MPC {
public:
    // A e B - Matrizes de Estado Discretas; Cc - Matriz de saídas restritas; Dc - Matriz de comandos restritos; Cr - Matriz de saídas reguladas
    Matrix A, B, Cc, Dc, Cr;

    // Matrizes de ponderação dos comandos e dos estados
    Matrix Qu, Qy;

    // Restrições das saídas restritas e da taxa de variação dos comandos restritos
    Matrix ycmax, ycmin, deltamax, deltamin;

    // Restrições dos comandos restritos
    Matrix umax, umin;

    // Construtor da classe
    MPC(MPCForm form = MPCForm::CLASSIC, int nWSR = 1000);

    // Funções com sobrecarga de método para Setup do MPC
    void compute_MPC_Matrices();
    void compute_MPC_Matrices(float* pontos);
    void compute_MPC_Matrices(float* lambda, float alpha, float tau);

    // Cálculo do sinal de comando
    float* compute_MPC_Command(float ulast, float* spt, float* err);

    private:
    
    // Ponteiro do otimizador qpOASES (Iniciado no construtor)
    qpOASES::QProblem *qp = nullptr;
    // Flag para guardar a inicialização do otimizador
    bool qp_initialized = false;
    // Quantidade de iterações do otimizador (Iniciado no construtor)
    int nWSR;
    
    // Indica qual foi a opção de MPC escolhida
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
    
    // Matrizes calculadas online
    qpOASES::real_t yref[N * ny];
    qpOASES::real_t F[N];
    qpOASES::real_t Bineq[2*n*N];
    qpOASES::real_t qp_opt[nU];
    
    // Matrizes de seleção para os casos parametrizados
    qpOASES::real_t Pi_r[nU * np];
    qpOASES::real_t Pi_e[nU * nre];

    // Matrizes para a parametrização trivial e exponencial
    qpOASES::real_t H_p[np * np];
    qpOASES::real_t Aineq_p[nAr * np];
    qpOASES::real_t F_p[np];
    qpOASES::real_t Bineq_p[nAr];
    
    //Matrizes de limite do sinal de comando
    qpOASES::real_t utildemax[nU];
    qpOASES::real_t utildemin[nU]; 

    // Matrizes auxiliares para armazenar o primeiro comando da sequência de sinal de comando calculada e a própria sequência
    qpOASES::real_t u_[nu];
    qpOASES::real_t u_full[nU];

    // Geração da trajetória desejada
    void generate_yref(const float* spt, qpOASES::real_t* yref);

    // Conversão da estrutura Matrix para Vetor Row-Major do tipo real_t
    void matrix_to_realt(const Matrix& M, qpOASES::real_t* result);

    // Calcula a priori as matrizes H, F1, F2, F3
    void compute_Cost_Matrices();
    
    // Calcula a priori as matrizes G1, G2, G3
    void compute_Constraints_Matrices();

    // Calcula a posteriori as matrizes Bineq e F, as quais dependem do estado x(k), ulast e yref
    void build_cost_vector(float* err);
    void build_constraints(float* err, float ulast);

    // Calcula o sinal de comando de saída
    void compute_util_opt();

    // Inicializa os parâmetros do solver QP
    void init_solver_qp(int size_qp);

    // Calcula o sinal de comando util para minimizar a função custo
    void solver_qp();

    // Calcula as matrizes reduzidas
    void compute_Bineq_reduced(qpOASES::real_t* Pi_ref);
    void compute_Aineq_reduced(qpOASES::real_t* Pi_ref);
    void compute_F_reduced(qpOASES::real_t* Pi_ref);
    void compute_H_reduced(qpOASES::real_t* Pi_ref);
    
    // Calcula as matrizes de seleção reduzidas (parametrização trivial e exponencial)
    void compute_Pi_e(float* lambda, float alpha, float tau);
    void compute_Pi_r(float* pontos);
};

#endif
