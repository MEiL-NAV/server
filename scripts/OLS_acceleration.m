% THETA = R11 R12 R13 b1 R21 R22 R23 b2 R31 R32 R33 b3

G = 9.805;
acc_true = [G -G 0  0 0  0;
            0  0 G -G 0  0;
            0  0 0  0 G -G];

acc_measured = [0.8*G -G 0  0 0  0;
                0  0 G -G 0  0;
                0  0 0  0 G -G];

Y = reshape(acc_true,[18,1])

X = zeros(18,12);

for i = 1:6
    X(3*(i-1) + 1, 1:4)  = [acc_measured(:,i)' 1];
    X(3*(i-1) + 2, 5:8)  = [acc_measured(:,i)' 1];
    X(3*(i-1) + 3, 9:12) = [acc_measured(:,i)' 1];
end

THETA = pinv(X' * X)* X' * YE

A = reshape(THETA,4,3)'



