function [Aineq, G1, G2, G3] = compute_constraits_matrices(MPC)

    Cc = MPC.Cc; N = MPC.N; A = MPC.A; B = MPC.B;

    if(isfield(MPC,'Dc') == 0)
        MPC.Dc = zeros(size(Cc, 1), size(B,2));
    end

    Dc = MPC.Dc;
    [n, nu] = size(B); inter_Psi_i = B; Phi_i = MPC.A;
    nc = size(Cc,1);

    Aineq_1=[]; Aineq_2=[];
    G1_1=[]; G3_11=[]; G3_12=[]; G3_21=[]; G3_22=[];
    interPinuN=zeros(N*nu,N*nu);

    for i=1:N
        Psi_i = [inter_Psi_i zeros(n,(N-i)*nu)];
        Pi_nuN = interPinuN((i-1)*nu+1:i*nu,:);
        Aineq_1 = [Aineq_1;Cc*Psi_i+Dc*Pi_nuN];
        ind1 = (i-1)*nu+1:i*nu;
        Aineq_2(ind1,ind1) = eye(nu);

        if (i>1)
            ind2 = (i-2)*nu+1:(i-1)*nu;
            Aineq_2(ind1,ind2) = -eye(nu);
        end

        G1_1 = [G1_1;-Cc*Phi_i];
        G3_11 = [G3_11; MPC.ycmax];
        G3_12 = [G3_12; -MPC.ycmin];
        G3_21 = [G3_21; MPC.deltamax];
        G3_22 = [G3_22; -MPC.deltamin];
        Phi_i = Phi_i*A;

        inter_Psi_i = [A*inter_Psi_i B];
    end

    Aineq_1 = [Aineq_1; -Aineq_1];
    Aineq_2 = [Aineq_2; -Aineq_2];

    Aineq = [Aineq_1; Aineq_2];
    G1_1 = [G1_1; -G1_1];
    G2_2 = [eye(nu); zeros((N-1)*nu,nu)];
    G2_2 = [G2_2; -G2_2];
    G3_1 = [G3_11; G3_12];
    G3_2 = [G3_21; G3_22];
    G1 = [G1_1; zeros(2*N*nu,n)];
    G2 = [zeros(2*N*nc,nu); G2_2];
    G3 = [G3_1; G3_2];

    return;

end