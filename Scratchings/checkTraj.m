function checkTraj(traj2D)
    len = length(traj2D.x);
    kappa_vec = zeros(1, len);
    sig_vec = zeros(1,len);
    for k = 1:len
        traj.q = [traj2D.x(k); traj2D.y(k)];
        traj.qdot = [traj2D.xdot(k); traj2D.ydot(k)];
        traj.qddot = [traj2D.xddot(k); traj2D.yddot(k)];
        traj.qdddot = [traj2D.xdddot(k); traj2D.ydddot(k)];
        %traj.qddddot = [traj2D.xddddot(k); traj2D.yddddot(k)];
        
        [v, w, kappa, sig] = calculateVelAndCurve(traj);
        kappa_vec(k) = kappa;
        sig_vec(k) = sig;
        
        if abs(kappa) > 0.36 || abs(sig) > 0.36
            warning('bad values');
        end
    end
    
    figure;
    subplot(2,1,1);
    plot(kappa_vec);
    
    subplot(2,1,2);
    plot(sig_vec);

    
end

function [v, w, k] = calculateVelocities(qdot, qddot)
%calculateVelocities
%
% Inputs:
%   qdot: \dot{q}, the time derivative of position (vel vector)
%   qddot: \ddot{q}, the second time derivative of position (accel vector)
%
% Outputs
%   v: translational velocity
%   w: rotational velocity
%   kappa: curvature
    %% Calculate using vector notation
    % Initialize variables
    J = [0 -1; 1 0]; %pi/2 rotation matrix

    % Calculate velocities
    v = sqrt(qdot'*qdot);
    w = -(qdot'*J*qddot)/(qdot'*qdot);
    
    % Calculate kurvature
    k = -(qdot'*J*qddot)*(qdot'*qdot)^(-3/2);
end

function [v, w, k, sig] = calculateVelAndCurve(traj)
%calculateVelocities
%
% Inputs:
%   qdot: \dot{q}, the time derivative of position (vel vector)
%   qddot: \ddot{q}, the second time derivative of position (accel vector)
%
% Outputs
%   v: translational velocity
%   w: rotational velocity
%   kappa: curvature
    %% Calculate using vector notation
    [psi, v, w, a, alpha] = getTrajectoryInformation(traj);
    
    k = w/v;
    sig = alpha/v - w/v^2*a;
end

function traj = getFollowerTrajFromLeaderTraj(traj_l, tau)
%getFollowerTrajFromLeaderTraj Returns the follower trajectory given the leader
%trajectory
%
% Inputs:
%   traj_l: Struct with leader trajectory information
%       .q = position
%       .qdot = velocity vector
%       .qddot = acceleration vector
%       .qdddot = jerk vector
%   tau: 2x1 nominal offset from the leader
%
% Outputs:
%   traj: Struct with follower trajectory information
%       .q = position
%       .qdot = velocity vector
%       .qddot = acceleration vector
%       .qdddot = jerk vector
    % Extract the leader trajectory information
    [psi_l, v_l, w_l, a_l, alpha_l] = getTrajectoryInformation(traj_l);
    R = [cos(psi_l), -sin(psi_l); sin(psi_l) cos(psi_l)];
    J = [0 -1; 1 0];
    
    % Calcualte the follower information
    traj.q = R*tau + traj_l.q;
    traj.qdot = w_l*R*J*tau + traj_l.qdot;
    traj.qddot = R*(w_l^2*J^2 + alpha_l*J)*tau + traj_l.qddot;
end

function [psi, v, w, a, alpha] = getTrajectoryInformation(traj)
%getTrajectoryInformation calcualte trajectory information directly from
%trajectory
%
% Inputs:
%   traj: Struct with trajectory information
%       .q = position
%       .qdot = velocity vector
%       .qddot = acceleration vector
%       .qdddot = jerk vector
%
% Outputs:
%   psi: orientation
%   v: translational velocity
%   w: rotational velocity
%   a: translational acceleration
%   alpha: rotational acceleration

    % Extract trajectory information
    xdot = traj.qdot(1); % Velocity vector
    ydot = traj.qdot(2);
    xddot = traj.qddot(1); % Accleration vector
    yddot = traj.qddot(2);
    xdddot = traj.qdddot(1); % Jerk vector
    ydddot = traj.qdddot(2);
    
    % Calculate the trajectgory variables
    psi = atan2(ydot, xdot);
    v = sqrt(xdot^2+ydot^2);
    w = 1/v^2*(xdot*yddot - ydot*xddot);
    a = (xdot*xddot + ydot*yddot)/v;
    alpha = (xdot*ydddot-ydot*xdddot)/v^2 - 2*a*w/v;    
end