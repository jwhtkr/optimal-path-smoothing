function EpsilonTracking()
close all;
    

    path = [0 4; 10 0; 0 -10; 10 -14; 20 -4];
%     path = -[0 0; 5 1; 10 -1; 15 1; 20 -1];
    k = 1;
    sig = 1;
    v = 1;
    dt = 0.01;
    clothoid = CCPathGenerator(path,v,dt,k,sig);
    %% Better Unicycle -- Diffeomorphism go to goal
    % Calculate point control gains
    A = [zeros(2) eye(2); zeros(2,4)]; 
    B = [zeros(2); eye(2)];
    Q = diag([10, 10, 10, 10]);
    R = diag([1, 1]);
    K = lqr(A, B, Q, R);
    
    % Create the vehicle and desired values
%     veh = BetterUnicycle;    
    veh = ContinuousSteeringBicycle;
    eps = .51;
     
    % Create reference trajectory
    a = 5; % Amplitude
    f = 1; % Frequency (in radians)
%     q_r = @(t)SineReference(t, a, f);
%     q_r = @(t)CircleReference(t,a,f);
    q_r = @(t)clothoid.reference_traj(t);
    q_d = @(t)EpislonTrajectory(t, q_r, eps);
    
    % Create the controller and initial conditions
    u = @(t,x)TrackTrajectoryApproximateDiffeomorphismUnicycle(t, x, veh, q_d, eps, K);
    x0 = [0; 0; 0; 0; 0];
    
    %% Simulate
    % Select the integration mode
    integrator = @(t0, dt, tf, x0, veh, u) integrateODE(t0, dt, tf, x0, veh, u);
    %integrator = @(t0, dt, tf, x0, veh, u) integratorEuler(t0, dt, tf, x0, veh, u);
    
    % Integrate the state
    t0 = 0; 
%     dt = 0.01;
    tf = 20;
    t = t0:dt:tf;
    for k = 1:length(t)
        epsilonTraj(:,k)= q_d(t(k));
    end
    [tmat, xmat] = integrator(t0, dt, tf, x0, veh, u);
       
    %% Plot the state in a simulation
    figure;
    
    % Plot the entire desired trajectory
    qd_traj = q_r(tmat);
    plot(qd_traj(1,:), qd_traj(2,:), 'r'); hold on;
    plot(epsilonTraj(1,:), epsilonTraj(2,:), 'b')
%     plot(path(:,1),path(:,2),'o')
%     ylim([-15 5])
%     axis equal
  
    pause();
    
    % Plot moving vehicle
    des_point = plot(qd_traj(1,1), qd_traj(2,1), 'ro', 'linewidth', 2);
    for k = 1:length(tmat)
       veh.plotState(tmat(k), xmat(:,k));
       set(des_point, 'xdata', qd_traj(1,k), 'ydata', qd_traj(2,k));
       pause(dt);
    end   
        
end

%%%%%%%%%%%%%%%%%%%%%%  Simulation Functions %%%%%%%%%%%%%%%%%%%%%%%%%%
function [tmat, xmat] = integrateODE(t0, dt, tf, x0, veh, u)
% Input parameters
%   t0: initial time
%   dt: time step for return data
%   tf: final time
%   x0: initial state
%   veh: instantiation of VehicleKinematics class
%   u: function handle which takes input arguments of (t,x)

    % Integrate forward in time
    [tmat, xmat] = ode45(@(t,x)veh.kinematics(t, x, u(t,x)), [t0:dt:tf], x0);
    
    % Transpose the outputs
    tmat = tmat';
    xmat = xmat';    
end

function [tmat, xmat] = integratorEuler(t0, dt, tf, x0, veh, u)
% Input parameters
%   t0: initial time
%   dt: time step for return data
%   tf: final time
%   x0: initial state
%   veh: instantiation of VehicleKinematics class
%   u: function handle which takes input arguments of (t,x)

    % Initialize state data
    tmat = [t0:dt:tf]';
    len = length(tmat);
    xmat = zeros(veh.dimensions, len);
    xmat(:,1) = x0;
    
    % Loop through and calculate the state
    x = x0;
    for k = 1:len
        % Calculate state update equation
        t = tmat(k);
        xdot = veh.kinematics(t, x, u(x,t));
        
        % Update the state
        x = x + dt * xdot;
        
        % Store the state
        xmat(:,k) = x;
    end

end

%%%%%%%%%%%%%%%%%%%%%% Reference trajectory functions %%%%%%%%%%%%%%%%%
function [qd, qd_dot, qd_ddot, qd_dddot] = SineReference(t, a, f)
    % Get length of the time vector
    n = length(t);

    % Get the reference
    qd = [t; a*sin(f.*t)];                     % Desired position
    qd_dot = [ones(1,n); a*f*cos(f.*t)];        % Desired velocity vector
    qd_ddot = [zeros(1,n); -a*f^2*sin(f.*t)]; % Desired acceleration vector
    qd_dddot = [zeros(1,n); -a*f^3*cos(f.*t)];
    
end

function [qd, qd_dot, qd_ddot, qd_dddot] = CircleReference(t, a, f)
    % Get length of the time vector
    n = length(t);

    % Get the reference
    qd = [a.*cos(f.*t); a.*sin(f.*t)];                     % Desired position
    qd_dot = [-a*f*sin(f*t); a*f.*cos(f.*t)];        % Desired velocity vector
    qd_ddot = [-a*f*f.*cos(f.*t); -a*f*f.*sin(f.*t)]; % Desired acceleration vector   
    qd_dddot = [a*f*f*f*sin(f*t); -a*f*f*f*cos(f*t)];
end

function [q_er, q_dot_er, q_ddot_er] = EpislonTrajectory(t, traj, eps)
    % Calculate Reference Trajectory
    [q_r, q_dot_r, q_ddot_r, q_dddot_r] = traj(t);
    
    % Calculate Reference Trajectory Parameters
    psi_r = atan2(q_dot_r(2),q_dot_r(1));
    v_r = norm(q_dot_r);
    w_r = (q_dot_r(1)*q_ddot_r(2) - q_dot_r(2)*q_ddot_r(1))/v_r^2;
    v = [v_r; w_r];
    a_r = (q_dot_r(1)*q_ddot_r(1) + q_dot_r(2)*q_ddot_r(2))/v_r;
    alpha_r = (q_dot_r(1)*q_dddot_r(2) - q_dot_r(2)*q_dddot_r(1))/v_r^2 - 2*a_r*w_r/v_r;
    a = [a_r; alpha_r];
    
    % Create Algebraic relations to epsilon trajectory
    R_er = [cos(psi_r) -eps*sin(psi_r); sin(psi_r) eps*cos(psi_r)];
    w_hat_r = [0 -eps*w_r; w_r/eps 0];
    
    % Create Epsilon Trajectory
    q_er = q_r + eps*[cos(psi_r);sin(psi_r)];
    q_dot_er = R_er*v;
    q_ddot_er = R_er*w_hat_r*v + R_er*a;
        
end

%%%%%%%%%%%%%%%%%%%%%% Control Functions %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function u = TrackTrajectoryApproximateDiffeomorphismUnicycle(t, x, veh, traj, eps0, K)
    % Calculate epsilon
    eps = eps0;

    % Get states
    x_pos = x(veh.x_ind);
    y_pos = x(veh.y_ind);
    [v, w] = veh.getVelocities(t, x, 0);
    th = x(veh.th_ind);
    c = cos(th);
    s = sin(th);
    
    % Form espilon variables
    w_hat_e = [0 -eps*w; w/eps 0];
    R_e = [c -eps*s; s eps*c];
    R_e_inv = [1 0; 0 1/eps] * [c s; -s c];
    
    % Calculate current values of espilon state
    q_eps = [x_pos; y_pos] + eps * [c; s];
    q_eps_dot = R_e*[v; w];
    q = [q_eps; q_eps_dot];

    % Get the desired trajectory
    [qd, qd_dot, qd_ddot] = traj(t);
        
    % Calculate point control
    u_point = -K*(q - [qd; qd_dot]) + qd_ddot;
    
    % Calculate the control inputs
    u = R_e_inv*u_point - w_hat_e*[v; w];  
end
