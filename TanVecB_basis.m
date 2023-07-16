function N_ = TanVecB_basis(u, U, i)
    N = zeros(3, 3);
    
    if u >= U(i+1) && u <= U(i + 2)
        N(1, 1) = 1;
    else
        N(1, 1) = 0;
    end
    
    if u >= U(i + 2) && u <= U(i + 3)
        N(2, 1) = 1;
    else
        N(2, 1) = 0;
    end
    
    if u >= U(i + 3) && u <= U(i + 4)
        N(3, 1) = 1;
    else
        N(3, 1) = 0;
    end
    
    for j = 2:3
        for k = 1:(3-j+1)
            deta1 = U(i+j+k-1) - U(i+k-1);
            deta2 = U(i+j+k-1) - U(i+k-1);
            A = 0;
            B1 = 0;
            if deta1 ~= 0
                A = (u - U(i+k)) / deta1;
            end
            if deta2 ~= 0
                B1 = (U(i+j+k-1) - u) / deta2;
            end
            N(k, j) = A * N(k, j-1) + B1 * N(k+1, j-1);
        end
    end
    
    N(3, 2) = 0;
    N(3, 3) = 0;
    N(2, 3) = 0;
    
    N_ = N(1, 3);
end
