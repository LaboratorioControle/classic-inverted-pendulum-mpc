function Pi_r=compute_Pi_r(lesN,N,nu)
    nr=length(lesN);
    np=nr*nu;
    Pi_r=zeros(N*nu,np);
    for i=1:N
        if (i==1)
            Pi_r(1:nu,1:nu)=eye(nu);
        elseif (i>=lesN(nr))
            Pi_r((i-1)*nu+1:i*nu,(nr-1)*nu+1:nr*nu)=eye(nu);
        else
            ji=find(lesN<=i, 1, 'last' );
            Pi_r((i-1)*nu+1:i*nu,(ji-1)*nu+1:ji*nu)=...
                (1-(i-lesN(ji))/(lesN(ji+1)-lesN(ji)))*eye(nu);
            Pi_r((i-1)*nu+1:i*nu,ji*nu+1:(ji+1)*nu)=...
                (i-lesN(ji))/(lesN(ji+1)-lesN(ji))*eye(nu);
        end
    end
    return;
end