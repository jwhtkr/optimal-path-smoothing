function [smoothed_traj, solver] = SmoothFollowerTraj(follower_traj, leader_traj, A_begin, b_begin, solver)
%SMOOTHFOLLOWERTRAJ Set up to smooth the followr traj with SmoothTrajOpt
%   @param follower_traj: The follower trajectory to be smoothed as a
%                        Trajectory2D object
%   @param leader_traj: the leader trajectory for reference (a Trajectory2D
%                       object)
%   @param A_begin: the position linear constraint A matrix (from Voronoi) 
%                   for this follower at time zero. Size: (p,2)
%   @param b_begin: the position linear constraint b vector (from Voronoi)
%                   for this follower at time zero. Size: (p,1)
%   @return smoothed_traj: the optimally smoothed trajectory as a
%                          Trajectory2D object
%
%   A_begin and b_begin are such that A_begin * [x;y] <= b_begin represent
%   the linear inequalities that bound the starting point of this follower
n = 2;
m = 5;
N = length(follower_traj.x);
xd_mat = zeros(n, m, N);
xd_mat(1,:,:) = [follower_traj.x; 
                 follower_traj.xdot; 
                 0*follower_traj.xddot; 
                 0*follower_traj.xdddot; 
                 0*follower_traj.xddddot];
xd_mat(2,:,:) = [follower_traj.y; 
                 follower_traj.ydot; 
                 0*follower_traj.yddot; 
                 0*follower_traj.ydddot; 
                 0*follower_traj.yddddot];

% plot(follower_traj.x, follower_traj.y); hold on; plot(leader_traj.x, leader_traj.y, 'LineWidth', 2);
% axis equal;
[A, b] = create_A_b(leader_traj, A_begin, b_begin, n, m, N);
% hold off;

Q = diag([1 1 0 0 10 10 10 10]);
R = diag([100 100]);
S = diag([1 1 0 0 0 0 0 0]);

dt = follower_traj.dt;

% Get optimally smoothed trajectory as a matrix
[smoothed_traj_mat, solver] = SmoothTrajOpt(xd_mat, Q, R, S, A, b, dt, solver);

% Convert to Trajectory2D object
smoothed_traj = Trajectory2D();
smoothed_traj.copy(follower_traj);  % Copy given traj to inherit unchanged member variables
% Squeeze and transpose to get to the right size: (1,N)
smoothed_traj.x = squeeze(smoothed_traj_mat(1,1,:))';
smoothed_traj.xdot = squeeze(smoothed_traj_mat(1,2,:))';
smoothed_traj.xddot = squeeze(smoothed_traj_mat(1,3,:))';
smoothed_traj.xdddot = squeeze(smoothed_traj_mat(1,4,:))';
smoothed_traj.xddddot = squeeze(smoothed_traj_mat(1,5,:))';
smoothed_traj.y = squeeze(smoothed_traj_mat(2,1,:))';
smoothed_traj.ydot = squeeze(smoothed_traj_mat(2,2,:))';
smoothed_traj.yddot = squeeze(smoothed_traj_mat(2,3,:))';
smoothed_traj.ydddot = squeeze(smoothed_traj_mat(2,4,:))';
smoothed_traj.yddddot = squeeze(smoothed_traj_mat(2,5,:))';
% Update the other trajectory vals based on x,y and their derivatives
smoothed_traj.updateTrajWithPositionalValues();
end

function R = R_leader(psi)
R = [cos(psi) -sin(psi); sin(psi) cos(psi)];
end

function [A, b] = create_A_b(leader_traj, A_begin, b_begin, n, m, N)
[p,~] = size(A_begin);  % get number of constraints, p

A_plus = (A_begin'*A_begin)\A_begin';

if p ~= 0
    A = zeros(p,n,m,N);
    b = zeros(p,N);
    quiv = [];
%     v = VideoWriter('traj_constraints.avi');
%     open(v)
    % Create A and b for each time step
    for i=1:N
        psi = leader_traj.psi(i);
        q = [leader_traj.x(i); leader_traj.y(i)];
        A(:,:,1,i) = transform_A(psi, A_begin);
        b(:,i) = transform_b(q, b_begin, A(:,:,1,i));
        
        % Validation
%         A_i = transform_A(psi, A_begin);
%         b_i = transform_b(q, b_begin, A_i);
%         A_plus_i = (A_i'*A_i)\A_i';
%         assert(all(A_plus_i - R_leader(psi)*A_plus < 1e-10, 'all'));
%         point = A_plus_i*b_i;
%         assert(norm(point - (A_plus_i*b_begin + q)) < 1e-10);
%         quiv = plot_constraint(A_i, b_i, 0.5, quiv);
%         if mod(i, 5) == 0
%             drawnow;
% %             writeVideo(v, getframe);
%         end
    end
else
    A = [];
    b = [];
end
% close(v);
end

function A = transform_A(psi, A0)
A = A0*R_leader(psi)';
end

function b = transform_b(q, b0, A_bar)
b = b0 + A_bar * q;
end

function quiv = plot_constraint(A, b, delta, quiv)
[p, ~] = size(A);
A_plus = (A'*A)\A';
q_center = A_plus*b;
points = repmat(q_center, 1, p) + delta.*A';
if isempty(quiv)
    quiv = quiver(points(1,:), points(2,:), A(:,1)', A(:,2)');
else
    set(quiv, 'XData', points(1,:), 'YData', points(2,:), 'UData', A(:,1)', 'VData', A(:,2)');
end
end
