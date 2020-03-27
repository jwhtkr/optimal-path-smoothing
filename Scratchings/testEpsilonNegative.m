function testEpsilonNegative
clc
    % Define matrix for block matrix
    I = eye(2);
    Z = zeros(2);
    A = [Z I; Z Z];
    B = [Z; I];
    
    % Define feedback matrix
    Q = eye(4);
    R = eye(2);
    %K = place(A, B, [-1, -2, -3, -4]);
    K = lqr(A, B, Q, R);
    
    % Look at A-BK
    Abar = A-B*K;
    lam = eig(Abar);
    
    sym = Abar - Abar';
    
    % Look at quadratic
    for k = 1:10000
        z = rand(4,1);
        err = z'*Abar*z;
        
        if err >= 0
            z
            warning('Not zero');
        end
    end
    
    display('finished');
    
end

