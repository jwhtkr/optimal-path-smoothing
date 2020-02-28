function kappaBounds
    % Test velocity calculation
%     testCalculateVelocities();
%     testCalculateFollowerTrajectory();
% %     getSymbolicCurvatureExpression();
%     testCalculateCurvature();
%     %testKmaxVelocity();
%     testZeroVelocityLemma();
    testMaxCurvature();
    
end

function testCalculateVelocities()
    for k = 1:1000
        qdot = rand(2,1) .* 10;
        qddot = rand(2,1) .* 10;
        calculateVelocities(qdot, qddot);
    end
end

function testCalculateFollowerTrajectory()
    for k = 1:1000
        % Get vl points
        q = rand(2,1);
        v = rand*10 + 1;
        psi_l = rand*10;
        kappa_l = rand*10;
        sigma_l = rand*10;
        
        tau = rand(2,1);
        
        % Get follower trajectory from vl trajectory point
        traj_l = getVLTrajPoint(q, v, psi_l, kappa_l, sigma_l);
        traj_f1 = getFollowerTrajFromLeaderTraj(traj_l, tau);
        
        % Get follower trajectory directly from vl state
        traj_f2 = getFollowerFromLeaderState(q, v, psi_l, kappa_l, sigma_l, tau);
        
        % Check error
        testTrajError(traj_f1, traj_f2);
    end
    
    function testTrajError(traj1, traj2)
        delta = 0.00000001;
        if norm(traj1.q - traj2.q) > delta
            error('q error');
        elseif norm(traj1.qdot - traj2.qdot) > delta
            error('qdot error');
        elseif norm(traj1.qddot - traj2.qddot) > delta
            error('qddot error');
        end            
    end
end

function getSymbolicCurvatureExpression()
    % Create a symbolic virtual leader trajectory point
    syms ql_1 ql_2 psi_l v_l kappa_l sigma_l real
    q_l = [ql_1; ql_2];
    
    % Create a symbolic offset vector
    syms tau_1 tau_2 real
    tau = [tau_1; tau_2];
    
    % Calculate the trajectory of the follower
    traj = getFollowerFromLeaderState(q_l, v_l, psi_l, kappa_l, sigma_l, tau);
    
    % Calculate the curvature
    [~, ~, kappa] = calculateVelocities(traj.qdot, traj.qddot);
    kappa = simplify(kappa)
    
end

function testCalculateCurvature()
    for k = 1:1000
        % Get vl points
        q_l = rand(2,1);
        psi_l = rand*10;
        v_l = rand*10 + 1;
        kappa_l = rand*10;
        sigma_l = rand*10;
        
        tau = rand(2,1) .* 10;
        
        % Calculate curvature direct from virtual leader
        kappa_f1 = getKappaFromLeaderState(v_l, kappa_l, sigma_l, tau);
        %kappa_f1 = getSymbolicKappaDerivation(v_l, kappa_l, sigma_l, tau(1), tau(2));
        
        % Calculate curvature by calculating the virtual leader trajectory
        traj_f = getFollowerFromLeaderState(q_l, v_l, psi_l, kappa_l, sigma_l, tau);
        [~, ~, kappa_f2] = calculateVelocities(traj_f.qdot, traj_f.qddot);
        
        if abs(kappa_f1 - kappa_f2) > .000000001
            error('error in follower curvature');
        end
    end
end

function testKmaxVelocity()
    
    % Create initial state
    q_l = rand(2,1);
    psi_l = rand*10;
    v_l = rand*10 + 1;
    sigma_l = rand*10;
    
    % Create a tau and curvature that will cause zero velocity
    tau = [0; 1];
    kappa_l = 1/tau(2);
    
    % Calculate 
    traj = getFollowerFromLeaderState(q_l, v_l, psi_l, kappa_l, sigma_l, tau);
    
    traj.qdot
    
    %% Create a symbolic virtual leader
    % Create a symbolic virtual leader trajectory point
    syms ql_1 ql_2 psi_l v_l kappa_l sigma_l real
    q_l = [ql_1; ql_2];
    
    % Create a symbolic offset vector
    syms tau_1 tau_2 real
    tau = [tau_1; tau_2];
    
    % Calculate the trajectory of the follower
    traj = getFollowerFromLeaderState(q_l, v_l, psi_l, kappa_l, sigma_l, tau);
    
    % Evaluate qdot'qdot
    qdot_nrm_sq = traj.qdot'*traj.qdot;
    
    % Evaluate by hand
    qdot_nrm_sq_hand = v_l^2*kappa_l^2*(tau'*tau) + v_l^2 - 2*v_l^2*kappa_l*tau_2;
    error_qdot_nrm_sq = simplify(qdot_nrm_sq - qdot_nrm_sq_hand)
    
    % Evaluate qdot/vl^2
    qdot_dv_vl = qdot_nrm_sq/(v_l^2);
    qdot_dv_vl_hand = kappa_l^2*tau'*tau + 1 - 2*kappa_l*tau_2;
    err_qdot_dv_v1 = simplify(qdot_dv_vl - qdot_dv_vl_hand)    
end

function testZeroVelocityLemma()

    % Initialize offset
    tau = rand(2,1) .* 2 - 1;
    %tau = [0; rand];
%     tau = [1; 0];
%     tau = [0; 1];
%     tau = 2*[1/sqrt(2); 1/sqrt(2)];
    
    
    % Initialize vl variables
    kappa_vec = [-10:.01:10, 1/tau(2)];
    q_l = rand(2,1);
    psi_l = 0;
    v_l = 1;
    sigma_l = rand*10 - 5;
    sigma_l = 3
    
    % Initialize figure
    figure; hold on;
    ax_qdot = gca;
    title('qdot');
    
    figure; hold on;
    ax_mag = gca;
    title('qdot mag');
    xlabel('curvature');
    ylabel('magnitude');
    
    figure; hold on;
    ax_psi = gca;
    title('psi');
    xlabel('curvature');
    ylabel('psi');
    
    figure; hold on;
    ax_curv = gca;
    title('follower curvature');
    xlabel('leader curvature');
    ylabel('follower');
    
    for kappa_l = kappa_vec
        % Calculate the trajectory point
        traj = getFollowerFromLeaderState(q_l, v_l, psi_l, kappa_l, sigma_l, tau);
        
        % Plot the velocity vector
        plot(ax_qdot, traj.qdot(1), traj.qdot(2), 'bo');        
        
        % Plot the magnitude
        plot(ax_mag, kappa_l, norm(traj.qdot), 'bo');
        
        % Plot the orientation
        plot(ax_psi, kappa_l, atan2(traj.qdot(2), traj.qdot(1)), 'bo');
        
        % Calculate the follower curvature
        kappa_f = getKappaFromLeaderState(v_l, kappa_l, sigma_l, tau);
        plot(ax_curv, kappa_l, kappa_f, 'bo');
    end
    plot(ax_qdot, tau(1), tau(2), 'ro');
    plot(ax_qdot, 0,0,'ko', 'linewidth', 3);
    
    % mark the value of curvature when kappa = 1/tau(2)
    kappa_f = getKappaFromLeaderState(v_l, 1/tau(2), sigma_l, tau);
    plot(ax_curv, 1/tau(2), kappa_f, 'ro', 'linewidth', 3);
    
    
    % mark the value of curvature when kappa = 1/tau(2)
    kappa_f = getKappaFromLeaderState(v_l, 1/tau(2), sigma_l, tau);
    plot(ax_curv, 1/tau(2), kappa_f, 'ro', 'linewidth', 3);
    
    % mark the value of curvature when kappa = 1/tau(2)
    kappa_f = getKappaFromLeaderState(v_l, 1/(2*tau(2)), sigma_l, tau);
    plot(ax_curv, 1/(2*tau(2)), kappa_f, 'go', 'linewidth', 3);

end

function testMaxCurvature()

    % Initialize offsets
    Q = [ [0;0], [-1.5; 1.5], [-1.5; -1.5], [-3; 0], [2; 0]];
    
    
    % Calculate clothoid
    max_k_foll = 1.25;
    max_k = 0.35;
    max_sigma = 0.35;
    u_max = 100;
    v_l = 1;
    dt = 0.01;
    curvature = SmoothCurvature(u_max, max_sigma, max_k);
    t_vec = curvature.getCurvatureTimeSpan(dt);
    x_vec_calc = curvature.calculateClothoidCurvature(t_vec);
    
    % Initialize vl variables
    kappa_vec = [-x_vec_calc(1,end:-1:1), x_vec_calc(1,:) ];
    sigma_vec = [-x_vec_calc(2,end:-1:1), x_vec_calc(2,:) ];
    t_vec = [t_vec, t_vec+t_vec(end)];
    
    % Create a figure of all the values
    % Initialize figure values
    figure('units','normalized','outerposition',[0 0 1 1]);;
    fontsize = 18;
    linewidth_all = 3;
    linewidth_foll = 2;
    
    % Calculate and plot the follower curvature vector for each follower
    subplot(3,1,1); hold on;
    n_follower = size(Q, 2);
    colors = distinguishable_colors(n_follower);
    for k = 1:n_follower
        % Calculate the curvature values
        kappa_f = calculateFollowerCurvature(kappa_vec, sigma_vec, v_l, Q(:,k));
        
        % Plot the follower curvature values
        plot(t_vec, kappa_f, 'color', colors(k,:), 'linewidth', linewidth_foll);        
    end
    ylabel('$\kappa_f$', 'Interpreter', 'latex');
    set(gca, 'fontsize', fontsize);
    
    % Plot the curvature bounds for follower
    plot([t_vec(1) t_vec(end)], [max_k_foll, max_k_foll], 'r:', 'linewidth', linewidth_foll);
    plot([t_vec(1) t_vec(end)], -[max_k_foll, max_k_foll], 'r:', 'linewidth', linewidth_foll);
    set(gca, 'ylim', [-max_k_foll - .25, max_k_foll + .25]);
    
    % Plot the curvature
    subplot(3,1,2);
    plot(t_vec, kappa_vec, 'b', 'linewidth', linewidth_all);
    ylabel('$\kappa_l$', 'Interpreter', 'latex');
    set(gca, 'fontsize', fontsize);
    
    % Plot the curvature rate
    subplot(3,1,3);
    plot(t_vec, sigma_vec, 'b', 'linewidth', linewidth_all);
    ylabel('$\sigma_l$', 'Interpreter', 'latex');
    xlabel('time (s)', 'Interpreter', 'latex');
    set(gca, 'fontsize', fontsize);    
end

function kappa_vec_f = calculateFollowerCurvature(kappa_vec, sigma_vec, v_l, tau)
%calculateFollowerCurvature Calculates the curvature of a follower with the
%leader executing a valid trajectory

    % Initialize the curvature
    len = length(kappa_vec);
    kappa_vec_f = zeros(1, len);
    
    % Loop through and calculate the curvature
    for k = 1:len
        % Extract the curvature values of the leader
        kappa_l = kappa_vec(k);
        sigma_l = sigma_vec(k);
        
        % Calculate the follower curvature
        kappa_vec_f(k) = getKappaFromLeaderState(v_l, kappa_l, sigma_l, tau);        
    end
end

function testCurvature()
    % Initialize offset
    tau = rand(2,1) .* 2 - 1;
        
    % Initialize vl variables
    kappa_vec = [-10:.01:10, 1/tau(2)];
    q_l = rand(2,1);
    psi_l = 0;
    v_l = 1;
    sigma_l = rand*10 - 5
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
    
    %% Recalculate using parameters
    % extract state
    xdot = qdot(1); ydot = qdot(2);
    xddot = qddot(1); yddot = qddot(2);
    
    % Calculate velocities
    v_par = sqrt(xdot^2 + ydot^2);
    w_par = 1/v^2 * (xdot*yddot - ydot*xddot);
    k_par = w_par/v_par;
    
    % Check values
    err_v = abs(v-v_par);
    err_w = abs(w-w_par);
    err_k = abs(k-k_par);
    
    delta = 0.00000000001;
    if err_v > delta || err_w > delta || err_k > delta
        error(['v_err = ' num2str(err_v) ' w_err = ' num2str(err_w) ' k_err = ' ...
            num2str(err_k)]);            
    end
end

function traj = getVLTrajPoint(q_l, v_l, psi_l, kappa_l, sigma_l) 
%getVLTrajPoint Returns the virtual leader trajectory given the trajectory
%parameters
%
% Inputs:
%   q_l: virtual leader position
%   v_l: Virtual leader velocity
%   psi_l: Virtual leader orientation
%   kappa_l: Virtual leader curvature
%   sigma_l: Virtual leader curvature change rate
%
% Outputs:
%   traj: Struct with trajectory information
%       .q = position
%       .qdot = velocity vector
%       .qddot = acceleration vector
%       .qdddot = jerk vector

    % Initialize variables
    J = [0 -1; 1 0]; %pi/2 rotation matrix
    h = [cos(psi_l); sin(psi_l)]; % Orientation vector
    
    % Calculate Trajectory information
    traj.q = q_l;
    traj.qdot = v_l*h;
    traj.qddot = v_l^2*kappa_l*J*h;
    traj.qdddot = v_l^2*sigma_l*J*h - v_l^3*kappa_l^2*h;    
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

function traj = getFollowerFromLeaderState(q_l, v_l, psi_l, kappa_l, sigma_l, tau)
%getFollowerFromLeader Returns the follower trajectory given the leader
%state
%
% Inputs:
%   q_l: virtual leader position
%   v_l: Virtual leader velocity
%   psi_l: Virtual leader orientation
%   kappa_l: Virtual leader curvature
%   sigma_l: Virtual leader curvature change rate
%
% Outputs:
%   traj: Struct with trajectory information for follower
%       .q = position
%       .qdot = velocity vector
%       .qddot = acceleration vector
%       .qdddot = jerk vector  
%   tau: 2x1 nominal offset from the leader

    % Calculate the virtual leader trajectory data
    R = [cos(psi_l), -sin(psi_l); sin(psi_l) cos(psi_l)];
    J = [0 -1; 1 0];
    h = [cos(psi_l); sin(psi_l)];
    
    % Calculate the position
    traj.q = R*tau + q_l;
    traj.qdot = kappa_l*v_l*R*J*tau + v_l*h;
    traj.qddot = R*(-v_l^2*kappa_l^2*eye(2) + v_l*sigma_l*J)*tau + v_l^2*kappa_l*J*h;
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

function kappa_f = getKappaFromLeaderState(v_l, kappa_l, sigma_l, tau)
%getKappaFromLeaderState calculates the follower curvature directly from
%the virtual leader state
%
% Inputs:
%   v_l: virtual leader velocity
%   kappa_l: virtual leader curvature
%   sigma_l: virtual leader curvature change rate
%   tau: 2x1 offset of follower from virtual leader
%
% Outputs:
%   kappa_f: curvature of the follower

    kappa_f = (v_l*kappa_l^3*(tau'*tau) + [sigma_l -2*v_l*kappa_l^2]*tau + v_l*kappa_l) / ...
        (v_l*(kappa_l^2*(tau'*tau) + [0 -2*kappa_l]*tau + 1)^(3/2));
end

function kappa = getSymbolicKappaDerivation(v_l, kappa_l, sigma_l, tau_1, tau_2)
    kappa = (v_l*kappa_l^3*tau_1^2 + v_l*kappa_l^3*tau_2^2 - 2*v_l*kappa_l^2*tau_2 + v_l*kappa_l + sigma_l*tau_1)/(abs(v_l)*(kappa_l^2*tau_1^2 + kappa_l^2*tau_2^2 - 2*kappa_l*tau_2 + 1)^(3/2));
end