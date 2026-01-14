function [J,g] = BBS(eta,param, dados)
    p=param.p;p(param.ell)=eta;N=param.N;
    lesu=p;tt=zeros(N+1,1);xx=zeros(4,N+1);
    uu=zeros(1,N+1);EE=zeros(N+1,1);
    xx(:,1)=param.x0;EE(1)=compute_E(param.x0, dados);

    for i=1:N
        uu(i)=lesu(i);
        EE(i+1)=compute_E(xx(:,i), dados);
        xx(:,i+1)=pendulum_one_step(xx(:,i)',uu(i),param.dt, dados);
    end

    J=sum(EE);
    g=max(abs(xx(3,:)))-param.rmax;
return