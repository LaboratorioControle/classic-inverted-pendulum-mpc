function f = P_i(i,n1,N)
    f = zeros(n1,N*n1);
    f(:,(i-1)*n1+1:i*n1) = eye(n1);
end