function [smoothed_traj, solver] = SmoothTrajOpt(traj_mat, Q, R, S, A_c_mat, b_c_mat, dt, solver)
%SMOOTHTRAJOPT Uses optimization to smooth an input trajectory
%   @param traj_mat: the input trajectory as a matrix of size: (n,m,N)
%                    where n is the dimensionality (2D, 3D, etc.), m is the 
%                    number of derivatives (determining the class of 
%                    smoothnes), and N is the number of time steps in the 
%                    trajectory.
%   @param Q: the instantaneous cost on nearness to input trajectory values
%             of size (n*(m-1) x n*(m-1))
%   @param R: the instantaneous cost on input values of size (nxn)
%   @param S: the terminal cost on nearness to last input trajectory value
%             of the same size as Q.
%   @param A_c: the linear inequality constraints for each time step as a
%               matrix of size: (p,n,m,N) with n,m,N as before and p the
%               number of constraints at each time instance.
%   @param b_c: the linear inequality constraints for each time step as a
%               matrix of size: (p,N) with p,N as previously defined.
%   @param dt: the time step size (for discretization)
%   @param solver: an optional pre-setup OSQP instance that just needs to
%                  be updated instead of completely re-created.
%
%   @return smoothed_traj: the optimally smoothed trajectory as a matrix
%                          the same size as traj_mat
%   @return solver: the setup solver (in case it is to be reused.
%
%   Note that the formulation of this problem is such that the vector y,
%   what we're optimizing over, is arranged like:
%       [q_1 qdot_1 ... q(m)_1 q_2 ... q(m)_2 ... q_N ... q(m)_N]'
%   where q(i)_j is the i-th derivative at time step j and is of size n. 
%   In terms of state (x_i) and control (u_i) this vector would then be:
%       [x_1 u_1 x_2 u_2 ... x_N u_N]'
%   Then, OSQP solves:
%       min_y 1/2 y'*P*y + q'*y
%           s.t. l <= A*y <= u
%       (Note that the solver, OSQP, uses the naming convention x for the 
%       optimization vector instead of y, and calls the dual solution y)

%% Setup
% Extract/calculate needed size values
[n, m, N] = size(traj_mat);
n_x = n*(m-1);
n_u = n;
n_state = n_x*(N);
n_ctrl = n_u*(N-1);

% Reshape passed in parameters to proper array dimensions for optimization
traj = reshape(traj_mat, [], 1);
[A_c, b_c] = reshape_Ac_bc(A_c_mat, b_c_mat);

% Extract initial condition, x0, from trajectory
x0 = traj(1:n_x);
% Extract terminal condition, xN, from trajectory
xN = traj(end-n_x-n_u+1:end-n_u);

%% Create dynamic constraints
% Create Continuous LTI dynamics
[A_dyn, B_dyn] = continuous_dynamics(n, m);
% Calculate exact discretization
[A_bar, B_bar] = exact_discretization(A_dyn, B_dyn, dt);
% Calculate A_eq and b_eq constraint matrices
[A_eq, b_eq] = equality_constraints(A_bar, B_bar, x0, xN, N);

%% Calculate P, q, A, l, and u for OSQP
[P, q] = calc_P_q_from_Q_R_S(Q, R, S, traj, n, m, N);
[A, l, u] = calc_constraints(A_c, b_c, A_eq, b_eq);

%% Setup OSQP
if isempty(solver)
    solver = osqp;
    solver.setup(P, q, A, l, u, 'verbose', false);
else
    [~,~,Px] = find(P);
    [~,~,Ax] = find(A);
    solver.update('Px', Px, 'Ax', Ax, 'q', q, 'l', l, 'u', u);
end

%% Solve with OSQP
% solver.warm_start('x', traj);
results = solver.solve();

% smooth_solve_time = results.info.run_time
smoothed_traj = reshape(results.x, n, m, N);
end

function [A_c, b_c] = reshape_Ac_bc(A_c_mat, b_c_mat)
% reshape_Ac_bc reshapes the input matrices to size needed for the solver.
%   @param A_c_mat: the linear inequality constraint matrix formatted as
%                   size (p,n,m,N)
%   @param b_c_mat: the linear inequality constraint vector formatted as a
%                   matrix of size (p,N)
%   @param N: the number of time steps, also the size of the 4th dim. of
%             A_c_mat and the second dim. of b_c_mat
%
%   @return A_c: the linear inequality constraint matrix formatted to be
%                compatible with the solver as size (p*N, n*m*N)
%   @return b_c: the linear inequality constraint vector formatted to be
%                compatible with the solver as size (p*N, 1)

% TODO: Reformulate with sparse methodology
[p,n,m,N] = size(A_c_mat);  % Extract size variables
A_c = spalloc(p*N, n*m*N, length(A_c_mat));    % Preallocate A_c

b_c = reshape(b_c_mat, [], 1);  % b_c is simple reshape to column vector

for i=1:N   
    % A_c requires looping through each time index as A_c_mat is in a 
    % compact form.
    
    % Calculate row and column indices based on i
    row_ind = (i-1)*p + 1;
    col_ind = (i-1)*n*m + 1;
    % Populate A_c with the reshaped A_c_mat for the i^th time instance
    A_c(row_ind:row_ind+p-1, col_ind:col_ind+n*m-1) = ...
        sparse(reshape(A_c_mat(:,:,:,i), p, n*m));
end
end

function [A_dyn, B_dyn] = continuous_dynamics(n, m)
% continuous_dynamics calculates the continuous LTI dynamic matrices
%   @param n: the number of dimensions
%   @param m: the numeber of derivatives (must be >1)
%
%   @return A_dyn: the A matrix of \dot{x} = Ax+Bu for a chain of
%                  integrators. Size: (n*(m-1), n*(m-1))
%   @return B_dyn: the B matrix of \dot{x} = Ax+Bu for a chain of
%                  integrators. Size: (n*(m-1), n)

I = eye(n); % Create identity for convenience
% Preallocate
A_dyn = zeros(n*(m-1));
B_dyn = zeros(n*(m-1), n);
for i=1:m-2
    % Create A_dyn of the form [Z I Z Z ... Z;
    %                           Z Z I Z ... Z;
    %                           . . . .     .;
    %                           . .   . .   .;
    %                           . .     . . .;
    %                           Z Z ...   Z I;
    %                           Z Z ...     Z]
    % with I=eye(n) and Z=zeros(n)
    
    % compute row and column indices
    row_ind = (i-1)*n + 1;
    col_ind = i*n + 1;
    
    % Fill A
    A_dyn(row_ind:row_ind+n-1, col_ind:col_ind+n-1) = I;
end
B_dyn(end-n+1:end,:) = I; % B_dyn of form [Z; Z; ... Z; I]
end

function [A_bar, B_bar] = exact_discretization(A_dyn, B_dyn, dt)
% exact_discretization computes the discretization of the CLTI dynamics
%   @param A_dyn: Continuous dynamics A matrix
%   @param B_dyn: Continuous dynamics B matrix
%
%   @return A_bar: Discrete dynamics A matrix
%   @return B_bar: Discrete dynamics B matrix
%
%   A naive (but slow-ish) approach is:
%   A_bar = expm(A_dyn*dt);
%   B_bar = integral(@(tau)expm(A_dyn*tau),0,dt, 'ArrayValued', true)*B_dyn;
%
%   However, a faster method is used that takes advantage of the property: 
%   expm([A B;0 0]*dt) = [A_bar B_bar;0 I]
%   (https://en.wikipedia.org/wiki/Discretization for more details)

% Extract size variables
[n_x,~] = size(A_dyn);
[~,n_u] = size(B_dyn);

% store result of expm([A B;0 0]*dt)
temp = expm([A_dyn B_dyn; zeros(n_u, n_x+n_u)]*dt);

% Extract A_bar and B_bar
A_bar = temp(1:n_x, 1:n_x);
B_bar = temp(1:n_x, n_x+1:end);
end

function [A_eq, b_eq] = equality_constraints(A_bar, B_bar, x0, xN, N)
% equality_constraints calculates constraints from the discrete dynamics
%   @param A_bar: the discrete dynamics A matrix from x_{k+1} = Ax_k + Bu_k
%   @param B_bar: the discrete dynamics B matrix from x_{k+1} = Ax_k + Bu_k
%   @param x0: the initial state of size (n,1)
%   @param xN: the terminal state, of size (n,1)
%   @param N: the number of time steps in the trajectory
%
%   @return A_eq: the equality constraint A matrix from Ay = b 
%   @return b_eq: the equality constraint b vector from Ay = b
%
%   Creates constraints for initial state, terminal state (start and end in
%   the same state as the desired trajectory) and the dynamics.
%
%   Uses a sparse matrix for speed and memory efficiency for A_eq, but can
%   be a little hard to follow the code. The main idea is to calculate
%   vectors for the index pairs, (i,j), and the corresponding value, v,
%   such that A(i(k), j(k)) = v(k) for k=1:n_nonzero.
%
%   In this code this corresponds to: 
%       A_eq(A_i(k), A_j(k)) = A_eq_v(k).

% Extract size variables
[n_x,~] = size(A_bar);
[~,n_u] = size(B_bar);

I = speye(n_x);   % Create identity matrix for convenience
ABI = [A_bar B_bar -I]; % Create convenience block matrix

% Find non_zero entries
[I_i, I_j, I_v] = find(I);
[ABI_i, ABI_j, ABI_v] = find(ABI);
nnz_I = length(I_v);
nnz_ABI = length(ABI_v);

% Preallocate space
b_eq = zeros((N+1)*n_x, 1);
A_i = zeros(2*nnz_I+(N-1)*nnz_ABI, 1);
A_j = zeros(size(A_i));

% First entries are different
A_i(1:nnz_I) = I_i;
A_j(1:nnz_I) = I_j;
b_eq(1:n_x) = -x0;

% A_eq_vec can be made in form [-I A B -I A B -I ... A B -I -I]
% with A = A_bar, B = B_bar and (N-1) sets of [A B -I] blocks
A_eq_v = [-I_v; repmat(ABI_v,N-1,1); -I_v];

for i=2:N
    % Calculate the indices for the rest of the time steps according to the
    % dynamics
    % Results in the form A_eq = [-I  0  0  0  0  0  ...   0  0;
    %                              A  B -I  0  0  0  ...   0  0;
    %                              0  0  A  B -I  0  ...   0  0;
    %                              .  .     .  .  .  .     .  .;
    %                              .  .        .  .  .  .  .  .;
    %                              .  .           .  .  .  .  .;
    %                              0  0     ...      A  B -I  0;
    %                              0  0     ...      0  0 -I  0]
    % with A = A_bar, B = B_bar.
    
    % calculate indices based on the time step
    row_ind = (i-1)*n_x + 1;
    col_ind = (i-2)*(n_x+n_u) + 1;
    ij_ind = (i-2)*nnz_ABI + nnz_I + 1;
    
    % add these shifted grids to the index arrays: A_i and A_j
    A_i(ij_ind:ij_ind+nnz_ABI-1) = ABI_i + row_ind - 1;
    A_j(ij_ind:ij_ind+nnz_ABI-1) = ABI_j + col_ind - 1;
end
% Last entries are different
A_i(end-nnz_I+1:end) = I_i + N*n_x;
A_j(end-nnz_I+1:end) = I_j + (N-1)*(n_x+n_u);
b_eq(end-n_x+1:end) = -xN;

A_eq = sparse(A_i, A_j, A_eq_v, (N+1)*n_x, N*(n_x+n_u));
end

function [P, q] = calc_P_q_from_Q_R_S(Q, R, S, xd, n, m, N)
% calc_P_q_from_Q_R_S computes P and q for the quadratic cost of OSQP
%   @param Q: the instantaneous cost on the state. Size: (n*(m-1), n*(m-1))
%   @param R: the instantaneous cost on the input. Size: (n, n)
%   @param S: the terminal cost on the state. Size: (n*(m-1), n*(m-1))
%   @param xd: the desired/nominal trajectory values as a column vector
%   @param n: the dimensionality of the trajectory
%   @param m: the number of derivatives of the trajectory
%   @param N: the number of time steps in the trajectory
%
%   @return P: the P matrix from the quadratic cost: x'Px+q'x
%   @return q: the q vector from the quadratic cost: x'Px+q'x
%
%   Uses a sparse matrix for P for speed and memory efficiency, but can be 
%   a little hard to follow the code. The main idea is to calculate vectors
%   for the index pairs, (i,j), and the corresponding value, v, such that
%   A(i(k), j(k)) = v(k) for k=1:n_nonzero. 
%
%   In this function this corresponds to:
%       P(P_i(k), P_j(k)) = P_vec(k)
%   P_i, P_j, and P_vec are calculated separately for the Q and R values
%   due to the difference in size of Q and R, but then are concatenated for
%   the creation of the sparse matrix P.

% Calculate number of x and u.
n_x = n*(m-1);
n_u = n;

% Find indices of non-zero entries
[Q_i, Q_j, Q_v] = find(Q);
[S_i, S_j, S_v] = find(S);
[R_i, R_j, R_v] = find(R);
nnz_QR = length(Q_v) + length(R_v);
nnz_S = length(S_v);

% Preallocate q and index vars
q = zeros(n*m*N, 1);
P_i = zeros((N-1)*nnz_QR + nnz_S, 1);
P_j = zeros(size(P_i));

% P_v can be calculated before as [Q_v; R_v; Q_v; R_v; ... S_v] with N-1
% sets of Q_v; R_v blocks.
QR_v = [Q_v; R_v];
P_v = [repmat(QR_v, N-1, 1); S_v];

for i=1:N-1
    % Create P of the form [Q 0 0 0 ...  0 0;
    %                       0 R 0 0 ...  0 0;
    %                       0 0 Q 0 ...  0 0;
    %                       0 0 0 R ...  0 0;
    %                       . .     .    . .;
    %                       . .       .  . .;
    %                       . .         .. .;
    %                       0 0   ...    S 0;
    %                       0 0   ...    0 0]
    %
    % And q of the form [-x_{d,1}Q 0 -x_{d,2}Q 0 ... -x_{d_N}S 0]
    
    % Calculate indices
    x_ind = (i-1)*n*m + 1;
    u_ind = x_ind + n_x;
    ij_ind = (i-1)*nnz_QR + 1;
    
    % Populate arrays
    P_i(ij_ind:ij_ind+nnz_QR-1) = [Q_i+x_ind-1; R_i+u_ind-1];
    P_j(ij_ind:ij_ind+nnz_QR-1) = [Q_j+x_ind-1; R_j+u_ind-1];
    q(x_ind:x_ind+n_x-1) = -xd(x_ind:x_ind+n_x-1)' * Q;
end
% Calculate last index (for S instead of Q on last state cost)
x_ind = (N-1)*n*m + 1;
ij_ind = (N-1)*nnz_QR + 1;

% Populate arrays
P_i(ij_ind:end) = S_i+x_ind-1;
P_j(ij_ind:end) = S_j+x_ind-1;
q(x_ind:x_ind+n_x-1) = -xd(x_ind:x_ind+n_x-1)' * S;

% Create Sparse P
P = sparse(P_i, P_j, P_v, N*n*m, N*n*m);
end

function [A, l, u] = calc_constraints(A_c, b_c, A_eq, b_eq)
% calc_constraints creates the linear inequality constraint parameters
%   @param A_c: the linear inequality constraints A matrix
%   @param b_c: the linear inequality constraints b vector
%   @param A_eq: the linear equality constraints A matrix
%   @param b_eq: the linear equality constraints b vector
%
%   @return A: the final, concatenated linear inequality constraint A
%              matrix
%   @return l: the final, concatenated linear inequality lower bound vector
%   @return u: the final, concatenated linear inequality upper bound vector
%
%   Takes in constraints of the form A_c*y <= b_c and A_eq*y = b_eq and
%   combines them in the form l <= A*y <= u

% Extract needed size variables
[p_c, ~] = size(A_c);

% Create arrays
A = [A_eq; A_c];
l = [b_eq; -inf*ones(p_c,1)];
u = [b_eq; b_c];
end