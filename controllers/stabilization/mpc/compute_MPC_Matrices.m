function MPC_new=compute_MPC_Matrices(MPC)
    MPC_new=MPC;
    [H,F1,F2,F3]=compute_cost_matrices(MPC);
    [Aineq,G1,G2,G3]=compute_constraits_matrices(MPC);
    MPC_new.H=H;MPC_new.F1=F1;MPC_new.F2=F2;MPC_new.F3=F3;
    MPC_new.Aineq=Aineq;
    MPC_new.G1=G1;
    MPC_new.G2=G2;
    MPC_new.G3=G3;
    utildemax=[]; utildemin=[];
    for i=1:MPC.N
        utildemax=[utildemax; MPC.umax];
        utildemin=[utildemin; MPC.umin];
    end
    MPC_new.utildemax=utildemax;
    MPC_new.utildemin=utildemin;
return