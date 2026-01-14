%% Calcula a solução de uma programação quadrática usando penalidades saturadas nas restrições

function [QP,lesp] = solver_sat_pen(QP)

    rho=QP.rho0;H=QP.H;F=QP.F;A=QP.A;B=QP.B;
    hmax0=norm(QP.H,2);hmaxg=norm(QP.A'*QP.A,2);
    np=size(H,1);indrho=1;
    lesp=zeros(np,QP.Niter);lesp(:,1)=QP.p0;
    beta_plus=1.8;beta_minus=0.4;
    gam_min=1.2;gam=gam_min;
    
    for i=1:QP.Niter-1
        hmax=hmax0+2*rho*hmaxg;
        if (indrho==QP.nrho)
            rho=min(QP.rho_max,QP.beta_plus_rho*rho);
            indrho=1;
        end
        inter=A*lesp(:,i)-B;
        G=H*lesp(:,i)+F+2*rho*A'*max(0,inter);
        p1=lesp(:,i)-1/(hmax)*G;
        p2=lesp(:,i)-gam/(hmax)*G;
        inter1=A*p1-B;inter2=A*p2-B;
        J1=0.5*p1'*H*p1+F'*p1+rho*(norm(max(0,inter1)))^2;
        J2=0.5*p2'*H*p2+F'*p2+rho*(norm(max(0,inter2)))^2;
        if (J1<J2)
            lesp(:,i+1)=p1;
            gam=max(gam_min,beta_minus*gam);
        else
            lesp(:,i+1)=p2;
            gam=beta_plus*gam;
        end
        for j=1:np
            lesp(j,i+1)=...
            min(QP.pmax(j),max(QP.pmin(j),lesp(j,i+1)));
        end
        indrho=indrho+1;
    end
    QP.rho0=rho;QP.psol=lesp(:,end);
    return;
end