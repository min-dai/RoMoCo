% File: simple_nlp.m

% Objective function: f(x) = x1^2 + x2^2
objective = @(x) (x(1)-.5)^2 + ((x(1)-.7)^2);

% Nonlinear constraint: x1^2 + x2 <= 1
nonlcon = @(x) deal( x(1)^2 + x(2) - 1, [] ); % inequality, equality

% Initial guess
x0 = [0.5];

% No linear constraints
A = []; b = [];
Aeq = []; beq = [];

% Variable bounds
lb = [0]; % x1, x2 >= 0
ub = [];     % No upper bounds

% Run the optimization
[x_opt, fval, exitflag, output] = fmincon(objective, x0, A, b, Aeq, beq, lb, ub, []);

% Display the result
fprintf('Optimal solution: x1 = %.4f', x_opt(1));
fprintf('Objective value: %.4f\n', fval);

%%
A = rand(2,5);
b = [1; 0];

if rank(A) == 2

x1 = pinv(A)*b

N = eye(5) - pinv(A)*A;





A2 = [A;
    rand(1,5)];
b2 = rand(3,1);

x2 = pinv(A2*N,0.00000001)*(b2-A2*x1)
x2 = lsqminnorm(A2*N, b2 - A2*x1,0.00000001)
end
%%

A = rand(2,5);            % Fat matrix
b = [1; 0];               % Desired Ax = b

x1 = pinv(A)*b;           % Particular solution

[U,S,V] = svd(A);         % Compute nullspace basis
N = V(:, rank(A)+1:end);  % Or use null(A) if available

A2 = [A; rand(1,5)];      % Add additional constraint
b2 = [b; 0.3];            % Ensure consistency with b

rhs = b2 - A2*x1;         % Residual to be explained by nullspace
x2 = pinv(A2*N) * rhs;    % Solve in nullspace coordinates

x = x1 + N*x2;            % Final solution satisfying both constraints

%%



clc
J0 = [            1  2.97612e-12            0    -0.148506    -0.774174  9.90639e-11    -0.671474   -0.0117377   -0.0298281    -0.341588    -0.047558  4.62684e-12            0            0            0            0            0            0            0            0            0            0            0            0            0            0
-2.97612e-12            1            0     0.127266  1.00858e-11     0.774174  8.15054e-12     0.641439    -0.147153  3.53915e-13  7.03175e-13         0.03            0            0            0            0            0            0            0            0            0            0            0            0            0            0
           0  3.82503e-28            1            0    -0.127266     0.148506    -0.127266    0.0298281   -0.0117377   -0.0603972        -0.12         0.03            0            0            0            0            0            0            0            0            0            0            0            0            0            0
           1  2.97612e-12            0   -0.0885065    -0.774174  6.31147e-11    -0.671474    0.0102331    0.0260046    -0.341588    -0.047558 -4.62684e-12            0            0            0            0            0            0            0            0            0            0            0            0            0            0
           0  3.82503e-28            1            0    -0.127266    0.0885065    -0.127266   -0.0260046    0.0102331   -0.0603972        -0.12        -0.03            0            0            0            0            0            0            0            0            0            0            0            0            0            0
           0  3.82503e-28            1            0    0.0427338     0.118506    0.0427338   0.00191176 -0.000752299     0.109603         0.05  1.48673e-13            0            0            0            0            0            0            0            0            0            0            0            0            0            0
           0            0            0            0   -0.0268731    -0.118424  -0.00336529  -0.00210899  0.000812468   -0.0577707 -0.000467205 -1.47188e-13   -0.0106315  0.000197233 -6.01682e-05   0.00183205 -0.000467205 -2.60272e-15  -0.00640031  0.000593424  4.02479e-07  -0.00575223  -0.00640031 -0.000593424 -4.02479e-07  -0.00575223
           0            0            0            0            0            0            0            0            1            0            0            0            0            0            0            0            0            0            0            0            0            0            0            0            0            0
           0            0            0            0            1  2.01948e-27            0            0            0            0            0            0            0            0            0            0            0            0            0            0            0            0            0            0            0            0
           0            0            0            0   2.8119e-21            1            0            0            0            0            0            0            0            0            0            0            0            0            0            0            0            0            0            0            0            0
           0            0            0     0.237013 -1.82931e-12 -1.42007e-10     0.671474  0.000752299   0.00191176     0.341588     0.047558  3.90834e-13    -0.671474  0.000752299   0.00191176    -0.341588    -0.047558  6.33989e-13            0            0            0            0            0            0            0            0
           0            0            0 -4.20477e-11  9.56526e-28  1.82943e-12  3.41018e-14    -0.597497     0.258818  4.79939e-12  6.19576e-13        -0.03 -3.41018e-14     0.597497    -0.258818   7.7926e-12  1.00504e-12         0.03            0            0            0            0            0            0            0            0
           0            0            0            0  4.20477e-11    -0.237013   0.00726619  -0.00191176  0.000752299   -0.0596028 -7.33478e-12 -1.48673e-13  -0.00726619  -0.00191176  0.000752299    0.0596028  8.96864e-12 -2.60708e-13            0            0            0            0            0            0            0            0
           0            0            0            0            0            0            0            0            0            0            0            0            0            0            1            0            0            0            0            0            0            0            0            0            0            0
           0            0            0            0            1  2.11331e-11            0            0            0            0            0            0            1  2.13838e-11  8.95897e-12            1            1  1.22451e-16            0            0            0            0            0            0            0            0
           0            0            0            0 -1.93719e-11     0.916667            0            0            0            0            0            0 -1.93719e-11     0.852999    -0.335665            0            0     0.916667            0            0            0            0            0            0            0            0];
b0 = [  4.24861e-14 -2.63763e-14 -2.95299e-13  4.27313e-14 -2.93247e-13 -2.94181e-13   4.2299e-08 -8.58898e-08 -9.82611e-06  7.69691e-08       10.581     -23.0594      20.7407  1.85726e-07 -1.16922e-06  4.93892e-08]';

eps = 0.000001;
ddq_raw = (pinv(J0,eps)*b0)'


x = quadprog(eye(26),[],[],[],J0,b0)
%%
w = [ones(1,6)*100. ones(1,6)*1 ones(1,6) ones(1,26-18)*1];
W = diag(w);

ddq_weighted = (weighted_pseudo_inverse(J0, w, 1e-6)*b0)'
%%


J1 = J0(7+4:end,:);
J0 = J0(1:6+4,:);
b1 = b0(7+4:end);
b0 = b0(1:6+4);

eps0 = 1e-6;
alpha0 = pinv(J0,eps0)*b0;
N0 = eye(26) - pinv(J0,eps0)*J0;
J1N0inv = pinv(J1*N0,eps);

alpha1 = J1N0inv * (b1 - J1 * alpha0);


ddq_target = alpha0 + N0 * alpha1;
ddq_target'



function result = weighted_pseudo_inverse(J, weights, threshold)
    % Function to compute the weighted pseudo-inverse solution
    % Inputs:
    %   J - The Jacobian matrix
    %   weights - A vector of weights
    %   threshold - Threshold for singular value truncation
    % Output:
    %   result - The weighted pseudo-inverse matrix

    % Create the weight matrix
    W = diag(weights);

    % Compute the weighted pseudo-inverse with threshold
    JWJ = J * pinv(W) * J';
    [U, S, V] = svd(JWJ);
    S_thresh = diag(max(diag(S), threshold)); % Apply threshold
    JWJ_inv = V * pinv(S_thresh) * U';

    result = pinv(W) * J' * JWJ_inv;
end
