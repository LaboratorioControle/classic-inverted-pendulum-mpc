function thc = truncate_theta(th)
    inter = zeros(2,1);
    inter(1) = abs(th) - 2*pi*fix(abs(th)/(2*pi));
    inter(2) = inter(1) - 2*pi;
    [~, ind] = min(abs(inter));
    thc = sign(th)*inter(ind);
end