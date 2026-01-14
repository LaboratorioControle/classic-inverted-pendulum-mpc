function Pi_e=compute_Pi_e(par_ex)
    tr=par_ex.tr;alpha=par_ex.alpha;ne=par_ex.ne;
    tau=par_ex.tau;N=par_ex.N;
    nu=length(ne);
    Pi_e=[];
    for i=0:N-1
        Mi=[];
        for j=1:nu
            Mji=zeros(1,ne(j));
            for ell=1:ne(j)
                Mji(ell)=...
                    exp(-2/tr(j)*(i*tau)/((ell-1)*alpha+1));
            end
            Mi=blkdiag(Mi,Mji);
        end
        Pi_e=[Pi_e;Mi];
    end
    return;
end