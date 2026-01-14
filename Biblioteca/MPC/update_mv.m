%---------------------------------------------------------
% Author : Mazen Alamir 
% CNRS, UNiversity of Grenoble-Alpes
% Gipsa Lab. Last modification March 2017
%---------------------------------------------------------

function p_sol=update_mv(mv,subset,param, dados)
coder.allowpcode('plain');
np=length(mv);
Nloop=max(2,fix((param.Nev-1)/(4*np*param.Niter)));
lesp=zeros(np,Nloop*np);
ind=1;
pmin=zeros(np,1);
pmax=zeros(np,1);
for i=1:np
    lesp(i,1)=mv(i).p;
    pmin(i)=mv(i).pmin;
    pmax(i)=mv(i).pmax;
end
npr=length(subset);
ell=subset(1);
for i=1:Nloop-1
    for sig=1:npr
        param.p=lesp(:,ind);param.ell=ell;
        param.pmin=pmin;param.pmax=pmax;
        mv(ell).p=lesp(ell,ind);
        mv(ell)=update_sc(mv(ell),param,param.Niter, dados);
        lesp(:,ind+1)=lesp(:,ind);lesp(ell,ind+1)=mv(ell).p;
        if (ell<npr)
            ell=subset(sig+1);
        else
            ell=subset(1);
        end
        ind=ind+1;
    end
end
lesp=lesp(:,1:ind-1);
p_sol=lesp(:,end);
return