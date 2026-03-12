#include "MPC.h"

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

    // tolerâncias mais rígidas
    options.terminationTolerance = 1e-8; 
    //options.boundTolerance = 1e-6;

    // melhora estabilidade numérica
    //options.enableRegularisation = qpOASES::BT_TRUE;
    //options.numRegularisationSteps = 1;

    options.printLevel = qpOASES::PL_NONE; 
    qp->setOptions(options);
}

void MPC::compute_MPC_Matrices(float* pontos){
    compute_Pi_r(pontos);
    
    compute_Cost_Matrices();
    compute_Constraints_Matrices();

    init_solver_qp(np);
}

void MPC::compute_MPC_Matrices(float* lambda, float alpha, float tau){
    compute_Pi_e(lambda, alpha, tau); 
    
    compute_Cost_Matrices();
    compute_Constraints_Matrices();

    init_solver_qp(np);
}

void MPC::compute_Cost_Matrices(){

    Matrix H_temp = Matrix::zeros(nU, nU);
    Matrix F1_temp = Matrix::zeros(nU, n);
    Matrix F2_temp = Matrix::zeros(nU, N*ny);
    Matrix F3_temp = Matrix::zeros(nU, nu);
    Matrix inter_Psi_i = B;
    Matrix Phi_i = A;

    for (int i = 1; i <= N; i++){

        // Psi_i = [inter_Psi_i zeros(n, (N-i)*nu)];
        Matrix Psi_i(n, nU, 0.0);
        Matrix::insertBlock(Psi_i, 0, 0, inter_Psi_i);

        Matrix Pi_nu_N = Matrix::P_i(i, nu, N);
        Matrix Pi_ny_N = Matrix::P_i(i, ny, N);

        // H = H + (Cr*Psi_i)'*Qy*Cr*Psi_i + Pi_nu_N'*Qu*Pi_nu_N;
        Matrix CrPsi = Matrix::mul(Cr, Psi_i);
        Matrix CrPsiT = Matrix::transpose(CrPsi);
        Matrix term1 = Matrix::mul(Matrix::mul(CrPsiT, Qy), CrPsi);
        Matrix PiTQuPi = Matrix::mul(Matrix::mul(Matrix::transpose(Pi_nu_N), Qu), Pi_nu_N);
        H_temp = H_temp + term1 + PiTQuPi;

        // F1 = F1 + Psi_i'*Cr'*Qy*Cr*Phi_i;
        Matrix termF1 = Matrix::mul(Matrix::mul(Matrix::transpose(Psi_i), Matrix::transpose(Cr)), Matrix::mul(Qy, Matrix::mul(Cr, Phi_i)));
        F1_temp = F1_temp + termF1;

        // F2 = F2 - Psi_i'*Cr'*Qy*Pi_ny_N;
        Matrix termF2 = Matrix::mul(Matrix::mul(Matrix::transpose(Psi_i), Matrix::transpose(Cr)), Matrix::mul(Qy, Pi_ny_N));
        F2_temp = F2_temp - termF2;

        // F3 = F3 + Pi_nu_N'*Qu;
        Matrix termF3 = Matrix::mul(Matrix::transpose(Pi_nu_N), Qu);
        F3_temp = F3_temp + termF3;

        Phi_i = Matrix::mul(Phi_i, A);

        // inter_Psi_i = [A*inter_Psi_i B];
        Matrix A_inter = Matrix::mul(A, inter_Psi_i);
        Matrix new_inter(n, inter_Psi_i.c + B.c, 0.0);
        Matrix::insertBlock(new_inter, 0, 0, A_inter);
        Matrix::insertBlock(new_inter, 0, A_inter.c, B);
        inter_Psi_i = new_inter;
    }

    // Conversão para vetores Row-Major
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
    
    if(Dc.r==0) 
        Dc = Matrix::zeros(Cc.r, B.c);
        
    // Declaração das matrizes de restrição auxiliares
    Matrix inter_Psi_i = B;
    Matrix Phi_i = A;

    // Pré-Alocação de Memória
    Matrix Aineq_1(2*N*nc,nU, 0.0);
    Matrix Aineq_2(2*N*nu,nU, 0.0);

    Matrix G1_1(2*N*nc, n, 0.0);
    Matrix G2_2(2*N*nu, nu, 0.0);
    Matrix G3_1(2*N*nc, 1, 0.0);
    Matrix G3_2(2*N*nu, 1, 0.0);

    int rowA1 = 0;
    int rowG1 = 0;

    for (int i = 1; i <= N ; i++){

        // Psi_i = [inter_Psi_i zeros(n,(N-i)*nu)];
        Matrix Psi_i(n, nU, 0.0);
        Matrix::insertBlock(Psi_i, 0, 0, inter_Psi_i);

        // Pi_nuN = interPinuN((i-1)*nu+1:i*nu,:);
        Matrix Pi_nuN(nu, nU, 0.0);

        // Aineq_1 = [Aineq_1;Cc*Psi_i+Dc*Pi_nuN];
        Matrix CcPsi_iDcPi_nuN = Matrix::mul(Cc,Psi_i) + Matrix::mul(Dc,Pi_nuN);
        Matrix::insertBlock(Aineq_1, rowA1, 0, CcPsi_iDcPi_nuN);
        rowA1 += nc;

        // ind1 = (i-1)*nu+1:i*nu;
        // Aineq_2(ind1,ind1) = eye(nu);
        Matrix::insertBlock(Aineq_2, (i-1)*nu, (i-1)*nu, Matrix::eye(nu));

        // if (i>1)
        //  ind2 = (i-2)*nu+1:(i-1)*nu;
        //  Aineq_2(ind1,ind2) = -eye(nu);
        // end
        if (i > 1){
            Matrix::insertBlock(Aineq_2, (i-1)*nu, (i-2)*nu, Matrix::smul(-1.0, Matrix::eye(nu)));
        }

        // G1_1 = [G1_1;-Cc*Phi_i];
        Matrix CcPhi_i= Matrix::smul(-1.0, Matrix::mul(Cc, Phi_i));
        Matrix::insertBlock(G1_1, rowG1, 0, CcPhi_i);
        rowG1 += nc;

        // G3_11 = [G3_11; MPC.ycmax];
        Matrix::insertBlock(G3_1, (i-1)*nc, 0, ycmax);
        // G3_12 = [G3_12; -MPC.ycmin];
        Matrix::insertBlock(G3_1, (N*nc) + (i-1)*nc, 0, Matrix::smul(-1.0, ycmin));

        // G3_21 = [G3_21; MPC.deltamax];
        Matrix::insertBlock(G3_2, (i-1)*nu, 0, deltamax);

        // G3_22 = [G3_22; -MPC.deltamin];
        Matrix::insertBlock(G3_2, (N*nu) + (i-1)*nu, 0, Matrix::smul(-1.0, deltamin));
       
        // Phi_i = Phi_i*A;
        Phi_i = Matrix::mul(Phi_i, A);

        // inter_Psi_i = [A*inter_Psi_i B];
        Matrix A_inter = Matrix::mul(A, inter_Psi_i);
        Matrix new_inter(n, inter_Psi_i.c + B.c, 0.0);
        Matrix::insertBlock(new_inter, 0, 0, A_inter);
        Matrix::insertBlock(new_inter, 0, A_inter.c, B);
        inter_Psi_i = new_inter;
    }


    // Aineq_1 = [Aineq_1; -Aineq_1];
    for(int i=0;i<N*nc;i++)
        for(int j=0;j<nU;j++)
            Aineq_1(N*nc + i, j) = -Aineq_1(i,j);

    // Aineq_2 = [Aineq_2; -Aineq_2];
    for(int i=0;i<N*nu;i++)
        for(int j=0;j<nU;j++)
            Aineq_2(N*nu + i, j) = -Aineq_2(i,j);

    // Aineq = [Aineq_1; Aineq_2];
    Matrix Aineq_temp(nA, nU, 0.0);
    Matrix::insertBlock(Aineq_temp, 0, 0, Aineq_1);
    Matrix::insertBlock(Aineq_temp, Aineq_1.r, 0, Aineq_2);

    // G1_1 = [G1_1; -G1_1];
    for(int i=0;i<N*nc;i++)
        for(int j=0;j<n;j++)
            G1_1(N*nc + i, j) = -G1_1(i,j);

    // G2_2 = [eye(nu); zeros((N-1)*nu,nu)];
    Matrix::insertBlock(G2_2, 0, 0, Matrix::eye(nu));

    // G2_2 = [G2_2; -G2_2];
    for(int i=0;i<N*nu;i++)
        for(int j=0;j<nu;j++)
            G2_2(N*nu + i, j) = -G2_2(i,j);


    // G1 = [G1_1; zeros(2*N*nu,n)];
    Matrix G1_temp(nA, n, 0.0);
    Matrix::insertBlock(G1_temp, 0, 0, G1_1);

    // G2 = [zeros(2*N*nc,nu); G2_2];
    Matrix G2_temp(nA, nu, 0.0);
    Matrix::insertBlock(G2_temp, 2*N*nc, 0, G2_2);

    // G3 = [G3_1; G3_2];
    Matrix G3_temp(nA, 1, 0.0);
    Matrix::insertBlock(G3_temp, 0, 0, G3_1);
    Matrix::insertBlock(G3_temp, G3_1.r, 0, G3_2);

    // Transformação do tipo Matrix para o tipo realt Row-Major
    matrix_to_realt(Aineq_temp, Aineq);
    matrix_to_realt(G1_temp, G1);
    matrix_to_realt(G2_temp, G2);
    matrix_to_realt(G3_temp, G3);

    if (form_ == MPCForm::LINEAR) {
        compute_Aineq_reduced(Pi_r);
        
    } else if(form_ == MPCForm::EXPONENCIAL){
        compute_Aineq_reduced(Pi_e);
    }

    // Computa as matrizes de restrição das variáveis de comando
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
    //    H_p[i*np + i] += 1e-6;
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

        } else if (i >= pontos[nre-1]) {
            // Pi_r((i-1)*nu+1:i*nu,(nr-1)*nu+1:nr*nu) = eye(nu)
            int col0 = (nre-1)*nu;
            for (int k = 0; k < nu; k++) {
                int row = row0 + k;
                int col = col0 + k;
                Pi_r[row*np + col] = 1.0;
            }

        } else {
            // ji = last index such that lesN[ji] <= i
            int ji = 0;
            for (int j = 0; j < nre; j++) {
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

void MPC::compute_Pi_e(float* lambda, float alpha, float tau){
    // Zera tudo
    for (int i = 0; i < N*nu*np; i++)
        Pi_e[i] = 0.0f;

    for (int i = 0; i < N; i++){
        int col_offset = 0;

        for (int u = 0; u < nu; u++){
            for (int j = 0; j < nre; j++){
                float denom = j * alpha + 1.0f;

                Pi_e[(i*nu+u)*np + col_offset + j] = expf(-2.0f / lambda[u] * (i * tau) / denom);
            }
            col_offset += nre;
        }
    }
}

void MPC::generate_yref(const float* spt) {
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

            solver_result_code = (int) ret;
        } else{
            qpOASES::returnValue ret = qp->hotstart(F,utildemin,utildemax,NULL,Bineq, nWSR_);

            solver_result_code = (int) ret;
        }

    } else if(form_ == MPCForm::LINEAR){

        compute_F_reduced(Pi_r);
        compute_Bineq_reduced(Pi_r);

        if(!qp_initialized){
            qpOASES::returnValue ret = qp->init(H_p,F_p,Aineq_p,NULL,NULL,NULL,Bineq_p, nWSR_);
            qp_initialized = true;

            solver_result_code = (int) ret;
        } else{
            qpOASES::returnValue ret = qp->hotstart(F_p,NULL,NULL,NULL,Bineq_p, nWSR_);

            solver_result_code = (int) ret;
        }

    } else if(form_ == MPCForm::EXPONENCIAL){

        compute_F_reduced(Pi_e);
        compute_Bineq_reduced(Pi_e);

        if(!qp_initialized){
            qpOASES::returnValue ret = qp->init(H_p,F_p,Aineq_p,NULL,NULL,NULL,Bineq_p, nWSR_);
            qp_initialized = true;

            solver_result_code = (int) ret;

        } else{
            qpOASES::returnValue ret = qp->hotstart(F_p,NULL,NULL,NULL,Bineq_p, nWSR_);

            solver_result_code = (int) ret;

            if(ret != qpOASES::SUCCESSFUL_RETURN) {
                Serial.print("QP Error Code: ");
                Serial.print((int)ret);
                Serial.print(" - ");
                Serial.println(qpOASES::MessageHandling::getErrorCodeMessage(ret));
            }
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
    
    generate_yref(spt);
    
    build_cost_vector(err);
    build_constraints(err, ulast);
    
    solver_qp();
    
    compute_util_opt();    
    
    return u_;
}