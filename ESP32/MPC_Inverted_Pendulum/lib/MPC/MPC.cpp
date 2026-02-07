#include "MPC.h"

Matrix eye(int n_){ Matrix I(n_,n_); for(int i=0;i<n_;i++) I(i,i)=1.0; return I; }
Matrix zeros(int r, int c){ return Matrix(r,c,0.0); }

Matrix transpose(const Matrix& A){ Matrix B(A.c, A.r); for(int i=0;i<A.r;i++) for(int j=0;j<A.c;j++) B(j,i)=A(i,j); return B; }

Matrix operator+(const Matrix& A, const Matrix& B){ assert(A.r==B.r && A.c==B.c); Matrix C(A.r,A.c); for(int i=0;i<A.r*A.c;i++) C.d[i]=A.d[i]+B.d[i]; return C; }
Matrix operator-(const Matrix& A, const Matrix& B){ assert(A.r==B.r && A.c==B.c); Matrix C(A.r,A.c); for(int i=0;i<A.r*A.c;i++) C.d[i]=A.d[i]-B.d[i]; return C; }

Matrix mul(const Matrix& A, const Matrix& B){ assert(A.c==B.r); Matrix C(A.r,B.c,0.0); for(int i=0;i<A.r;i++) for(int k=0;k<A.c;k++){ float aik=A(i,k); for(int j=0;j<B.c;j++) C(i,j)+=aik * B(k,j); } return C; }

Matrix smul(float s, const Matrix& A){ Matrix C(A.r,A.c); for(int i=0;i<A.r*A.c;i++) C.d[i]=s*A.d[i]; return C; }

void insertBlock(Matrix& dst, int r0, int c0, const Matrix& src){
    assert(r0+src.r <= dst.r && c0+src.c <= dst.c);
    for(int i=0;i<src.r;i++) for(int j=0;j<src.c;j++) dst(r0+i,c0+j)=src(i,j);
}

Matrix P_i(int i, int n1, int N_){
    Matrix f(n1, N_*n1, 0.0);
    // block columns (i-1)*n1 to i*n1-1
    for(int r=0;r<n1;r++) f(r,(i-1)*n1 + r) = 1.0;
    return f;
}

MPC::MPC(MPCForm form, int nWSR){
    this->nWSR = nWSR;
    this->form_ = form;
}

void MPC::compute_MPC_Matrices(){
    compute_Cost_Matrices();
    compute_Constraints_Matrices();

    init_solver_qp(nU);
}

void MPC::init_solver_qp(int size_qp){
    qp = new qpOASES::QProblem(size_qp,nc);

    qpOASES::Options options;
    options.setToMPC();
    options.terminationTolerance = 1e-4; 
    options.printLevel = qpOASES::PL_NONE; 
    qp->setOptions(options);
}

void MPC::compute_MPC_Matrices(float* pontos){
    if (form_ == MPCForm::LINEAR) {
        compute_Pi_r(pontos);
    } else if(form_ == MPCForm::EXPONENCIAL){
        compute_Pi_e(pontos);
    }
    
    compute_Cost_Matrices();
    compute_Constraints_Matrices();

    init_solver_qp(np);
}

void MPC::compute_Cost_Matrices(){
    int nH = N*nu;
    Matrix H_temp = zeros(nH, nH);
    Matrix F1_temp = zeros(nH, n);
    Matrix F2_temp = zeros(nH, N*ny);
    Matrix F3_temp = zeros(nH, nu);
    Matrix inter_Psi_i = B;
    Matrix Phi_i = A;

    for(int i=1;i<=N;i++){

        // Psi_i = [inter_Psi_i zeros(n, (N-i)*nu)];
        Matrix Psi_i(n, N*nu, 0.0);
        insertBlock(Psi_i, 0, 0, inter_Psi_i);
        Matrix Pi_nu_N = P_i(i, nu, N);
        Matrix Pi_ny_N = P_i(i, ny, N);

        // H = H + (Cr*Psi_i)'*Qy*Cr*Psi_i + Pi_nu_N'*Qu*Pi_nu_N;
        Matrix CrPsi = mul(Cr, Psi_i);
        Matrix CrPsiT = transpose(CrPsi);
        Matrix term1 = mul(mul(CrPsiT, Qy), CrPsi);
        Matrix PiTQuPi = mul(mul(transpose(Pi_nu_N), Qu), Pi_nu_N);
        H_temp = H_temp+ term1 + PiTQuPi;

        // F1 = F1 + Psi_i'*Cr'*Qy*Cr*Phi_i;
        Matrix termF1 = mul(mul(transpose(Psi_i), transpose(Cr)), mul(Qy, mul(Cr, Phi_i)));
        F1_temp = F1_temp + termF1;

        // F2 = F2 - Psi_i'*Cr'*Qy*Pi_ny_N;
        Matrix termF2 = mul(mul(transpose(Psi_i), transpose(Cr)), mul(Qy, Pi_ny_N));
        // subtract
        for(int rr=0; rr<F2_temp.r; ++rr) for(int cc=0; cc<F2_temp.c; ++cc) F2_temp(rr,cc) -= termF2(rr,cc);

        // F3 = F3 + Pi_nu_N'*Qu;
        Matrix termF3 = mul(transpose(Pi_nu_N), Qu);
        F3_temp = F3_temp + termF3;
        // update
        Phi_i = mul(Phi_i, A);

        // inter_Psi_i = [A*inter_Psi_i B]; horizontally appended
        Matrix A_inter = mul(A, inter_Psi_i);
        Matrix new_inter(n, inter_Psi_i.c + B.c, 0.0);
        insertBlock(new_inter, 0, 0, A_inter);
        insertBlock(new_inter, 0, A_inter.c, B);
        inter_Psi_i = new_inter;

    }

    matrix_to_realt(H_temp, H);
    matrix_to_realt(F1_temp, F1);
    matrix_to_realt(F2_temp, F2);
    matrix_to_realt(F3_temp, F3);

    if (form_ == MPCForm::LINEAR) {
        compute_H_reduced(Pi_r);
    } else if(form_ == MPCForm::EXPONENCIAL){
        compute_H_reduced(Pi_e);
    }
}

void MPC::compute_Constraints_Matrices(){
    // Translates compute_constraits_matrices
    if(Dc.r==0) Dc = zeros(Cc.r, B.c);
    std::vector<float> interPinuN(N*nu*N*nu, 0.0); // large zero matrix flattened (but we only slice)
    Matrix inter_Psi_i = B;
    Matrix Phi_i = A;
    Matrix Aineq_1(0,0), Aineq_2(0,0);
    Matrix G1_1(0, n);
    std::vector<float> G3_11_v, G3_12_v, G3_21_v, G3_22_v;
    // We'll accumulate rows into std::vector< vector<double> > and convert at end
    std::vector<std::vector<float>> rows_Aineq1;
    std::vector<std::vector<float>> rows_Aineq2;
    std::vector<std::vector<float>> rows_G1_1;
    for(int i=1;i<=N;i++){
        // Psi_i = [inter_Psi_i zeros(n,(N-i)*nu)];
        Matrix Psi_i(n, N*nu, 0.0);
        insertBlock(Psi_i, 0, 0, inter_Psi_i);
        // Pi_nuN slice: MATLAB used a big zero matrix interPinuN and slices rows
        // In this translation Pi_nuN acts like zeros; so Pi_nuN is zeros(n, N*nu) except maybe later usage
        Matrix Pi_nuN(n, N*nu, 0.0);
        // row Aineq1: Cc*Psi_i + Dc*Pi_nuN
        Matrix rowA = mul(Cc, Psi_i);
        // convert rowA to rows
        for(int rr=0; rr<rowA.r; ++rr){
            std::vector<float> row(rowA.c);
            for(int cc=0; cc<rowA.c; ++cc) row[cc] = rowA(rr,cc);
            rows_Aineq1.push_back(row);
        }
        // Aineq_2 building: banded identity and -identity between blocks (we'll build full matrix later)
        // we'll push identity rows incrementally
        // Build block row for this i of size N*nu (one-hot for this nu-block and negative for previous)
        std::vector<float> row_id(N*nu, 0.0);
        for(int j=0;j<nu;j++) row_id[(i-1)*nu + j] = 1.0;
        rows_Aineq2.push_back(row_id);
        if(i>1){
            std::vector<float> row_prev(N*nu, 0.0);
            for(int j=0;j<nu;j++) row_prev[(i-2)*nu + j] = -1.0;
            // add to same row? MATLAB sets in columns; equivalent is to OR them: sum
            for(int k=0;k<N*nu;k++) rows_Aineq2.back()[k] += row_prev[k];
        }
        // G1_1 accumulate: -Cc*Phi_i (each adds nc x n rows)
        Matrix G1row = smul(-1.0, mul(Cc, Phi_i));
        for(int rr=0; rr<G1row.r; ++rr){
            std::vector<float> rown(n);
            for(int cc=0; cc<n; ++cc) rown[cc] = G1row(rr,cc);
            rows_G1_1.push_back(rown);
        }
        // G3 pieces are constraints bounds stacking
        // MPC.ycmax, ycmin, deltamax, deltamin are assumed column vectors (nc x 1 or nu x1)
        // stack them as scalars per row; in MATLAB they just concatenated the whole vectors
        for(int rr=0; rr<ycmax.r; ++rr) G3_11_v.push_back(ycmax(rr,0));
        for(int rr=0; rr<ycmin.r; ++rr) G3_12_v.push_back(-ycmin(rr,0));
        for(int rr=0; rr<deltamax.r; ++rr) G3_21_v.push_back(deltamax(rr,0));
        for(int rr=0; rr<deltamin.r; ++rr) G3_22_v.push_back(-deltamin(rr,0));
        // update
        Phi_i = mul(Phi_i, A);
        Matrix A_inter = mul(A, inter_Psi_i);
        Matrix new_inter(n, inter_Psi_i.c + B.c, 0.0);
        insertBlock(new_inter, 0, 0, A_inter);
        insertBlock(new_inter, 0, A_inter.c, B);
        inter_Psi_i = new_inter;
    }
    // After loop: Aineq_1 = [Aineq_1; -Aineq_1]; Aineq_2 = [Aineq_2; -Aineq_2]
    int rows1 = rows_Aineq1.size();
    int cols1 = (rows1>0? rows_Aineq1[0].size():0);
    Matrix A1(rows1*2, cols1, 0.0);
    for(int i=0;i<rows1;i++) for(int j=0;j<cols1;j++) A1(i,j) = rows_Aineq1[i][j];
    for(int i=0;i<rows1;i++) for(int j=0;j<cols1;j++) A1(rows1+i,j) = -rows_Aineq1[i][j];
    int rows2 = rows_Aineq2.size();
    int cols2 = (rows2>0? rows_Aineq2[0].size():0);
    Matrix A2(rows2*2, cols2, 0.0);
    for(int i=0;i<rows2;i++) for(int j=0;j<cols2;j++) A2(i,j) = rows_Aineq2[i][j];
    for(int i=0;i<rows2;i++) for(int j=0;j<cols2;j++) A2(rows2+i,j) = -rows_Aineq2[i][j];

    // Combine
    Matrix Aineq_temp = Matrix(A1.r + A2.r, A1.c, 0.0);
    // insert A1 at top
    insertBlock(Aineq_temp, 0, 0, A1);
    // insert A2 after A1
    insertBlock(Aineq_temp, A1.r, 0, A2);
    // G1_1 doubled and then G1 built with zeros bottom
    int g1rows = rows_G1_1.size();
    Matrix G1dup(g1rows*2, n, 0.0);
    for(int i=0;i<g1rows;i++) for(int j=0;j<n;j++) G1dup(i,j) = rows_G1_1[i][j];
    for(int i=0;i<g1rows;i++) for(int j=0;j<n;j++) G1dup(g1rows+i,j) = -rows_G1_1[i][j];
    // G2_2 = [eye(nu); zeros((N-1)*nu, nu)]; then duplicated with negative
    Matrix G2_2(2*N*nu, nu, 0.0);
    // first block
    for(int i=0;i<nu;i++) G2_2(i,i)=1.0;
    // bottom negative block
    for(int i=0;i<nu;i++) G2_2(N*nu + i, i) = -1.0;
    // G3 build
    // In MATLAB G3 = [G3_1; G3_2]; where G3_1 stacks ycmax and -ycmin, G3_2 stacks deltamax and -deltamin
    std::vector<float> G3_1_v; G3_1_v.insert(G3_1_v.end(), G3_11_v.begin(), G3_11_v.end()); G3_1_v.insert(G3_1_v.end(), G3_12_v.begin(), G3_12_v.end());
    std::vector<float> G3_2_v; G3_2_v.insert(G3_2_v.end(), G3_21_v.begin(), G3_21_v.end()); G3_2_v.insert(G3_2_v.end(), G3_22_v.begin(), G3_22_v.end());
    // G1 = [G1_1; zeros(2*N*nu,n)];
    Matrix G1_temp = Matrix(G1dup.r + 2*N*nu, n, 0.0);
    insertBlock(G1_temp, 0, 0, G1dup);
    // remaining rows zero already
    // G2 = [zeros(2*N*nc,nu); G2_2];
    Matrix G2_temp = Matrix(2*N*nc + G2_2.r, nu, 0.0);
    insertBlock(G2_temp, 2*N*nc, 0, G2_2);
    // G3 = [G3_1; G3_2];
    int G3rows = G3_1_v.size() + G3_2_v.size();
    Matrix G3_temp = Matrix(G3rows, 1, 0.0);
    for(int i=0;i<G3_1_v.size();++i) G3_temp(i,0) = G3_1_v[i];
    for(int i=0;i<G3_2_v.size();++i) G3_temp(G3_1_v.size()+i, 0) = G3_2_v[i];

    matrix_to_realt(Aineq_temp, Aineq);
    matrix_to_realt(G1_temp, G1);
    matrix_to_realt(G2_temp, G2);
    matrix_to_realt(G3_temp, G3);

    if (form_ == MPCForm::LINEAR) {
        compute_Aineq_reduced(Pi_r);
        
    } else if(form_ == MPCForm::EXPONENCIAL){
        compute_Aineq_reduced(Pi_e);
    }


    int k = 0;
    for (int i = 0; i < N; i++) {
        for (int j = 0; j < nu; j++) {
            utildemax[k] = umax(j,0);
            utildemin[k] = umin(j,0);
            k++;
        }
    }
}


// Matrizes reduzidas para parametrização
void MPC::compute_H_reduced(qpOASES::real_t* Pi_ref){
    for (int i = 0; i < np; i++) {
        for (int j = 0; j < np; j++) {
            float sum = 0.0f;

            for (int k = 0; k < nU; k++) {
                for (int l = 0; l < nU; l++) {
                    sum +=
                        Pi_ref[k*np + i] *
                        H[k*nU + l] *
                        Pi_ref[l*np + j];
                }
            }

            H_p[i*np + j] = sum;
        }
    }

    //for (int i = 0; i < np; i++)
    //    H_p[i*np + i] += 1e-2;
}

void MPC::compute_F_reduced(qpOASES::real_t* Pi_ref){
    for (int i = 0; i < np; i++) {
        float sum = 0.0f;
        for (int k = 0; k < nU; k++)
            sum += Pi_ref[k*np + i] * F[k];

        F_p[i] = sum;
    }
}

void MPC::compute_Aineq_reduced(qpOASES::real_t* Pi_ref){
    int row = 0;

    // Aineq * Pi_r
    for (int i = 0; i < nA; i++) {
        for (int j = 0; j < np; j++) {
            float sum = 0.0f;
            for (int k = 0; k < nU; k++)
                sum += Aineq[i*nU + k] * Pi_ref[k*np + j];

            Aineq_p[row*np + j] = sum;
        }
        row++;
    }

    // -Pi_r
    for (int i = 0; i < nU; i++, row++) {
        for (int j = 0; j < np; j++)
            Aineq_p[row*np + j] = -Pi_ref[i*np + j];
    }

    // Pi_r
    for (int i = 0; i < nU; i++, row++) {
        for (int j = 0; j < np; j++)
            Aineq_p[row*np + j] = Pi_ref[i*np + j];
    }
}

void MPC::compute_Bineq_reduced(qpOASES::real_t* Pi_ref){
    int row = 0;

    // Bineq
    for (int i = 0; i < nA; i++)
        Bineq_p[row++] = Bineq[i];

    // -utildemin
    for (int i = 0; i < nU; i++)
        Bineq_p[row++] = -utildemin[i];

    // utildemax
    for (int i = 0; i < nU; i++)
        Bineq_p[row++] = utildemax[i];
}

void MPC::compute_Pi_r(float* pontos){
    // zera tudo
    for (int i = 0; i < N*nu*np; i++)
        Pi_r[i] = 0.0;

    for (int i = 1; i <= N; i++) {   // MATLAB-style pra facilitar tradução
        int row0 = (i-1)*nu;

        if (i == 1) {
            // Pi_r(1:nu,1:nu) = eye(nu)
            for (int k = 0; k < nu; k++) {
                int row = row0 + k;
                int col = k;
                Pi_r[row*np + col] = 1.0;
            }

        } else if (i >= pontos[nr-1]) {
            // Pi_r((i-1)*nu+1:i*nu,(nr-1)*nu+1:nr*nu) = eye(nu)
            int col0 = (nr-1)*nu;
            for (int k = 0; k < nu; k++) {
                int row = row0 + k;
                int col = col0 + k;
                Pi_r[row*np + col] = 1.0;
            }

        } else {
            // ji = last index such that lesN[ji] <= i
            int ji = 0;
            for (int j = 0; j < nr; j++) {
                if (pontos[j] <= i)
                    ji = j;
            }

            float alpha =
                1.0 - float(i - pontos[ji]) /
                float(pontos[ji+1] - pontos[ji]);

            float beta =
                float(i - pontos[ji]) /
                float(pontos[ji+1] - pontos[ji]);

            // bloco ji
            for (int k = 0; k < nu; k++) {
                int row = row0 + k;
                int col = ji*nu + k;
                Pi_r[row*np + col] = alpha;
            }

            // bloco ji+1
            for (int k = 0; k < nu; k++) {
                int row = row0 + k;
                int col = (ji+1)*nu + k;
                Pi_r[row*np + col] = beta;
            }
        }
    }
}

void MPC::compute_Pi_e(float* pontos){

}

void MPC::generate_yref(const float* spt, qpOASES::real_t* yref) {
    for (int k = 0; k < N; k++) {
        for (int j = 0; j < ny; j++) {
            yref[k * ny + j] = spt[j];
        }
    }
}

void MPC::matrix_to_realt(const Matrix& M, qpOASES::real_t* result) {    
    int k = 0;

    for (int i = 0; i < M.r; i++) {
        for (int j = 0; j < M.c; j++) {
            result[k++] = static_cast<qpOASES::real_t>(M(i, j));
        }
    }
}

void MPC::build_cost_vector(float* err){
    // F = MPC.F1*err' + MPC.F2*yref_pred;
    for (int i = 0; i < N*nu; i++) {
        F[i] = 0.0f;

        // F1 * err
        for (int j = 0; j < n; j++) {
            F[i] += F1[i*n + j] * err[j];
        }

        // F2 * yref
        for (int j = 0; j < N*ny; j++) {
            F[i] += F2[i*(N*ny) + j] * yref[j];
        }
    }
}

void MPC::build_constraints(float* err, float ulast){
    //Bineq = MPC.G1*err' + MPC.G2*MPC.ulast + MPC.G3;
    for (int i = 0; i < nA; i++) {
        Bineq[i] = 0.0f;

        // G1 * err
        for (int j = 0; j < n; j++) {
            Bineq[i] += G1[i*n + j] * err[j];
        }

        // G2 * ulast
        for (int j = 0; j < nu; j++) {
            Bineq[i] += G2[i*nu + j] * ulast;
        }

        // + G3
        Bineq[i] += G3[i];
    }
}

void MPC::solver_qp(){

    int nWSR_ = nWSR;
    
    if (form_ == MPCForm::CLASSIC) {

        if(!qp_initialized){
            qpOASES::returnValue ret = qp->init(H,F,Aineq,utildemin,utildemax,NULL,Bineq, nWSR_);
            qp_initialized = true;
        } else{
            qpOASES::returnValue ret = qp->hotstart(F,utildemin,utildemax,NULL,Bineq, nWSR_);
        }

    } else if(form_ == MPCForm::LINEAR){

        compute_F_reduced(Pi_r);
        compute_Bineq_reduced(Pi_r);

        if(!qp_initialized){
            qpOASES::returnValue ret = qp->init(H_p,F_p,Aineq_p,NULL,NULL,NULL,Bineq_p, nWSR_);
            qp_initialized = true;
        } else{
            qpOASES::returnValue ret = qp->hotstart(F_p,NULL,NULL,NULL,Bineq_p, nWSR_);
        }

    } else if(form_ == MPCForm::EXPONENCIAL){

        compute_F_reduced(Pi_r);
        compute_Bineq_reduced(Pi_r);

        if(!qp_initialized){
            qpOASES::returnValue ret = qp->init(H_p,F_p,Aineq_p,NULL,NULL,NULL,Bineq_p, nWSR_);
            qp_initialized = true;
        } else{
            qpOASES::returnValue ret = qp->hotstart(F_p,NULL,NULL,NULL,Bineq_p, nWSR_);
        }

    }

    qp->getPrimalSolution(qp_opt);
}

void MPC::compute_util_opt(){

    if (form_ == MPCForm::CLASSIC) {
        for (int i = 0; i < nu; i++) 
            u_[i] = qp_opt[i];
            
    } else if(form_ == MPCForm::LINEAR){
        for (int i = 0; i < N*nu; i++) {
            u_full[i] = 0.0;
            for (int j = 0; j < np; j++) {
                u_full[i] += Pi_r[i*np + j] * qp_opt[j];
            }
        }

        for (int i = 0; i < nu; i++) {
            u_[i] = u_full[i];
        }

    } else if(form_ == MPCForm::EXPONENCIAL){
        for (int i = 0; i < N*nu; i++) {
            u_full[i] = 0.0;
            for (int j = 0; j < np; j++) {
                u_full[i] += Pi_e[i*np + j] * qp_opt[j];
            }
        }

        for (int i = 0; i < nu; i++) {
            u_[i] = u_full[i];
        }
    }
}

float* MPC::compute_MPC_Command(float ulast, float* spt, float* err){
    
    generate_yref(spt, yref);
    
    build_cost_vector(err);
    build_constraints(err, ulast);
    
    solver_qp();
    
    compute_util_opt();    
    
    return u_;
}

#include <fstream>
#include <iostream>

#ifdef MPC_TEST_MAIN
int main(){

    std::ofstream file("matrizes_mpc.csv");
    std::streambuf* oldCout = std::cout.rdbuf(); // guarda cout original
    std::cout.rdbuf(file.rdbuf());               // redireciona para o arquivo

    #ifndef M_PI
    #define M_PI 3.14159265358979323846
    #endif

    MPC mpc;

    // =========================
    // MATRIZES DO MODELO
    // =========================
    mpc.A = Matrix(4,4);
    mpc.A(0,0)=1.0000; mpc.A(0,1)=0.0000; mpc.A(0,2)=0.0091;  mpc.A(0,3)=0.0000;
    mpc.A(1,0)=0.0000; mpc.A(1,1)=1.0022; mpc.A(1,2)=-0.0039; mpc.A(1,3)=0.0100;
    mpc.A(2,0)=0.0000; mpc.A(2,1)=0.0044; mpc.A(2,2)=0.8224;  mpc.A(2,3)=0.0000;
    mpc.A(3,0)=0.0000; mpc.A(3,1)=0.4346; mpc.A(3,2)=-0.7526; mpc.A(3,3)=1.0021;

    mpc.B = Matrix(4,1);
    mpc.B(0,0)=0.0000;
    mpc.B(1,0)=0.0001;
    mpc.B(2,0)=0.0068;
    mpc.B(3,0)=0.0288;

    // =========================
    // MATRIZ DE SAÍDA
    // =========================
    mpc.Cr = Matrix(2,4);
    mpc.Cr(0,0)=1; mpc.Cr(0,1)=0; mpc.Cr(0,2)=0; mpc.Cr(0,3)=0;
    mpc.Cr(1,0)=0; mpc.Cr(1,1)=1; mpc.Cr(1,2)=0; mpc.Cr(1,3)=0;

    mpc.Cc = Matrix(3,4);
    mpc.Cc(0,0)=1; mpc.Cc(0,1)=0; mpc.Cc(0,2)=0; mpc.Cc(0,3)=0;
    mpc.Cc(1,0)=0; mpc.Cc(1,1)=1; mpc.Cc(1,2)=0; mpc.Cc(1,3)=0;
    mpc.Cc(2,0)=0; mpc.Cc(2,1)=0; mpc.Cc(2,2)=1; mpc.Cc(2,3)=0;

    // =========================
    // PESOS DO MPC
    // =========================
    mpc.Qy = zeros(2,2);
    mpc.Qy(0,0) = 50;
    mpc.Qy(1,1) = 10;

    mpc.Qu = Matrix(1,1);
    mpc.Qu(0,0) = 0.001;

    mpc.N = 35;

    // =========================
    // LIMITES
    // =========================
    float pos_limite = 20.0/100.0;
    float ang_limite = 12.0 * (M_PI/180.0);
    float vel_limite = 45.0/100.0;
    float comando_limite = 12.0;

    mpc.ycmax = Matrix(3,1);
    mpc.ycmax(0,0)= pos_limite;
    mpc.ycmax(1,0)= ang_limite;
    mpc.ycmax(2,0)= vel_limite;

    mpc.ycmin = Matrix(3,1);
    mpc.ycmin(0,0)= -pos_limite;
    mpc.ycmin(1,0)= -ang_limite;
    mpc.ycmin(2,0)= -vel_limite;

    mpc.umax = Matrix(1,1); 
    mpc.umax(0,0) = comando_limite;

    mpc.umin = Matrix(1,1); 
    mpc.umin(0,0) = -comando_limite;

    mpc.deltamax = Matrix(1,1); 
    mpc.deltamax(0,0) = 1e5;

    mpc.deltamin = Matrix(1,1); 
    mpc.deltamin(0,0) = -1e5;

    // =========================
    // CALCULA MATRIZES
    // =========================
    mpc.compute_MPC_Matrices();

    // =========================
    // PRINT
    // =========================
    //mpc.printMatrix("H", mpc.H);
    //mpc.printMatrix("F1", mpc.F1);
    //mpc.printMatrix("F2", mpc.F2);
    //mpc.printMatrix("F3", mpc.F3);
    //mpc.printMatrix("Aineq", mpc.Aineq);
    //mpc.printMatrix("G1", mpc.G1);
    //mpc.printMatrix("G2", mpc.G2);
    //mpc.printMatrix("G3", mpc.G3);

    std::cout.rdbuf(oldCout);  // restaura o cout pro normal
    file.close();

    return 0;
}
#endif



// FIM
