function ret = polyFit(x, y, k)
    n = length(x);
    Y = zeros(n, 1);
    U = zeros(n, k+1);
    trans_U = zeros(k+1, n);
    Inv_U = zeros(k+1, k+1);
    M_U = zeros(k+1, k+1);
    TM_U = zeros(k+1, n);
    M_Y = zeros(k+1, 1);

    for i = 1:n
        for j = 0:k
            U(i, j+1) = x(i)^j;
        end
        Y(i) = y(i);
    end

    trans_U = U';
    M_U = trans_U * U;
    Inv_U = inv(M_U);
    TM_U = Inv_U * trans_U;
    M_Y = TM_U * Y;

    ret = M_Y';
end
