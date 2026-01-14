function MPCr = compute_reduced_matrices(MPC,Pi_r)
    MPCr=MPC;
    MPCr.H=Pi_r'*MPC.H*Pi_r;
    MPCr.F=Pi_r'*MPC.F;
    MPCr.Aineq=[MPC.Aineq*Pi_r;-Pi_r;Pi_r];
    MPCr.Bineq=[MPC.Bineq;-MPC.utildemin;MPC.utildemax];
    return;
end