function [H, F1, F2, F3] = compute_cost_matrices(MPC)

A = MPC.A; B = MPC.B; N = MPC.N;
Qu = MPC.Qu; Qy = MPC.Qy; Cr = MPC.Cr;

[n, nu] = size(B);

inter_Psi_i = B;
Phi_i = A;

ny = size(Cr, 1);
nH = N*nu;

H = zeros(nH,nH);
F1 = zeros(nH,n);
F2 = zeros(nH,N*ny);
F3 = zeros(nH,nu);

for i=1:N
    Psi_i = [inter_Psi_i zeros(n, (N-i)*nu)];
    Pi_nu_N = P_i(i, nu, N);
    Pi_ny_N = P_i(i, ny, N);

    H = H+(Cr*Psi_i)'*Qy*Cr*Psi_i+Pi_nu_N'*Qu*Pi_nu_N;

    F1 = F1+Psi_i'*Cr'*Qy*Cr*Phi_i;

    F2 = F2-Psi_i'*Cr'*Qy*Pi_ny_N;

    F3 = F3+Pi_nu_N'*Qu;
    
    Phi_i = Phi_i*A;
    inter_Psi_i = [A*inter_Psi_i B];
end

return;

end