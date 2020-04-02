function smoothed_traj = SmoothTrajOpt(traj_mat, Q, R, S, A_c_mat, b_c_mat)
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
%   @return smoothed_traj: the optimally smoothed trajectory as a matrix
%                          the same size as traj_mat
%   Note that the formulation of this problem is such that the vector y,
%   what we're optimizing over, is arranged like:
%       [q_0 qdot_0 ... q(m)_0 q_1 ... q(m)_1 ... q_N ... q(m)_N]'
%   where q(i)_j is the i-th derivative at time step j and is of size n. 
%   In terms of state (x_i) and control (u_i) this vector would then be:
%       [x_0 u_0 x_1 u_1 ... x_N u_N]'
%   Then, OSQP solves:
%       min_y 1/2 y'*P*y + q'*y
%           s.t. l <= A*y <= u
%       (Note that the solver, OSQP, uses the naming convention x for the 
%       optimization vector instead of y, and calls the dual solution y)
%% Setup
% Extract/calculate needed size values
[n, m, N] = traj_mat.size();
n_x = n*(m-1);
n_u = n;
n_state = n_x*(N+1);
n_ctrl = n_u*N;

% Reshape passed in parameters to proper array dimensions for optimization
traj = reshape(traj_mat, [], 1);
[A_c, b_c] = reshape_Ac_bc(A_c_mat, b_c_mat);

%% Create dynamic constraints
% Create Continuous LTI dynamics
[A_dyn, B_dyn] = continous_dynamics(n, m);
% Calculate exact discretization
[A_bar, B_bar] = exact_discretization(A_dyn, B_dyn);
% Calculate A_eq and b_eq constraint matrices
[A_eq, b_eq] = equality_constraints(A_bar, B_bar, N);

%% Calculate P, q, A, l, and u for OSQP
[P, q] = calc_P_q_from_Q_R_S(Q, R, S);
[A, l, u] = calc_constraints(A_c, b_c, A_eq, b_eq);

%% Setup OSQP
solver = osqp;
solver.setup(P, q, A, l, u, 'verbose', False);

%% Solve with OSQP
solver.warm_start('x', traj);
results = solver.solve();

smoothed_traj = reshape(results.x, n, m, N);
end