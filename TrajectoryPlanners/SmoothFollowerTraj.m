function smoothed_traj = SmoothFollowerTraj(follower_traj, leader_traj, A_begin, b_begin)
%SMOOTHFOLLOWERTRAJ Set up to smooth the followr traj with SmoothTrajOpt
%   @param follower_traj: The follower trajectory to be smoothed as a
%                        Trajectory2D object
%   @param leader_traj: the leader trajectory for reference
%   @param A_begin: the position linear constraint A matrix (from Voronoi) 
%                   for this follower at time zero.
%   @param b_begin: the position linear constraint b vector (from Voronoi)
%                   for this follower at time zero.\
%   @return smoothed_traj: the optimally smoothed trajectory as a
%                          Trajectory2D object
%
%   A_begin and b_begin are such that A_begin * [x;y] <= b_begin represent
%   the linear inequalities that bound the starting point of this follower
n = 2;
m = 5;
N = length(follower_traj.x);
xd_mat = zeros(n, m, N);
xd_mat(1,:,:) = [follower_traj.x; follower_traj.xdot; follower_traj.xddot; 
                 follower_traj.xdddot; follower_traj.xddddot];
xd_mat(2,:,:) = [follower_traj.y; follower_traj.ydot; follower_traj.yddot; 
                 follower_traj.ydddot; follower_traj.yddddot];

[A, b] = create_A_b(leader_traj, A_begin, b_begin, n, m, N);

Q = diag([1 1 0 0 0 0 0 0 0 0]);
R = diag([1 1]);
S = diag([1 1 0 0 0 0 0 0 0 0]);

% Get optimally smoothed trajectory as a matrix
smoothed_traj_mat = SmoothTrajOpt(xd_mat, Q, R, S, A, b);

% Convert to Trajectory2D object
smoothed_traj = Trajectory2D();
smoothed_traj.x = smoothed_traj_mat(1,1,:);
smoothed_traj.xdot = smoothed_traj_mat(1,2,:);
smoothed_traj.xddot = smoothed_traj_mat(1,3,:);
smoothed_traj.xdddot = smoothed_traj_mat(1,4,:);
smoothed_traj.xddddot = smoothed_traj_mat(1,5,:);
smoothed_traj.y = smoothed_traj_mat(2,1,:);
smoothed_traj.ydot = smoothed_traj_mat(2,2,:);
smoothed_traj.yddot = smoothed_traj_mat(2,3,:);
smoothed_traj.ydddot = smoothed_traj_mat(2,4,:);
smoothed_traj.yddddot = smoothed_traj_mat(2,5,:);
% Update the other trajectory vals based on x,y and their derivatives
smoothed_traj.updateTrajWithPositionalValues();
end

function R = R_leader(psi)
R = [cos(psi) -sin(psi); sin(psi) cos(psi)];
end

function [A, b] = create_A_b(leader_traj, A_begin, b_begin, n, m, N)
[p,~] = size(A_begin);  % get number of constraints, p

A = zeros(p,n,m,N);
b = zeros(p,N);
A(:,:,1,1) = A_begin;
b(:,1) = b_begin;
psi_0 = leader_traj.psi(0);
% Create A and b for each time step
for i=2:N
    psi = leader_traj.psi(i);
    del_q = [leader_traj.x(i) - leader_traj.x(1); 
             leader_traj.y(i) - leader_traj.y(1)];
    A(:,:,1,i) = transform_A(psi-psi_0, A_begin);
    b(:,i) = transform_b(del_q, b_begin, A(:,:,1,i));
end
end

function A = transform_A(del_psi, A0)
A = A0*R_leader(del_psi)';
end

function b = transform_b(del_q, b0, A_bar)
b = b0 + A_bar * del_q;
end
