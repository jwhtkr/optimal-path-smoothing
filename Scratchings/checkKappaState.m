function checkKappaState
    % Set time values for switching on bang-bang control
    umax = 100; % max control input
    sig_max = 5; % Max value for sigma
    kappa_max = 5; % Maximum desired curvature value
    dt = 0.001;
    
    % Compute the clothoid curvature
    curvature = computeClothoidCurvature(umax, sig_max, kappa_max);
    [t_vec, x_vec_calc] = curvature.calculateClothoidCurvature(dt);
    [t_switch, u_switch] = curvature.extractSwitchTimesAndControl();
    
    % Simulate forward in time
    [t_vec_sim, x_vec_sim] = ode45(@(t,x) kappaStateSwitchedDynamics(t, x, t_switch, u_switch), t_vec, zeros(3,1));
    x_vec_sim = x_vec_sim';
    
    %% Plot the results
    figure('units','normalized','outerposition',[0 0 1 1]);
    subplot(3,1,1); % Plot the kappa state
    plot(t_vec, x_vec_calc(1,:), 'r', 'linewidth', 3); hold on;
    plot(t_vec_sim, x_vec_sim(1,:), 'b', 'linewidth', 2);
    ylabel('\kappa');
    
    subplot(3,1,2); % Plot the sigma state
    plot(t_vec, x_vec_calc(2,:), 'r', 'linewidth', 3); hold on;
    plot(t_vec_sim, x_vec_sim(2,:), 'b', 'linewidth', 2);
    ylabel('\sigma');
    
    subplot(3,1,3); % Plot the gamma state
    plot(t_vec, x_vec_calc(3,:), 'r', 'linewidth', 3); hold on;
    plot(t_vec_sim, x_vec_sim(3,:), 'b', 'linewidth', 2);
    ylabel('\gamma');
    
end

function xdot = kappaStateSwitchedDynamics(t, x_k, t_switch, u_switch)
    
    % find the time and control
    i = find(t < t_switch, 1, 'first');
    
    % Calculate dynamics
    if isempty(i)
        xdot = kappaStateDynamics(t, x_k, 0);
    else
        xdot = kappaStateDynamics(t, x_k, u_switch(i));
    end
end
function xdot = kappaStateDynamics(t, x_k, u)
%kappaStateDynamics returns the time derivative of the kappa state at time
%t given the state x_k
%
% The kappa state is a 3x1 vector:
%         [kappa]
%   x_k = [sigma]
%         [gamma]
%
% The time derivative is given by
%   d/dt(kappa) = sigma
%   d/dt(sigma) = gamma
%   d/dt(gamma) = u
%
% Inputs:
%   t: time
%   x_k: the kappa state at time t
%   u: the input at time t
%
% Outputs:
%   xdot: d/dt(x_k)

    % Extract states
%     kappa = x_k(1);
    sigma = x_k(2);
    gamma = x_k(3);
    
    % Calcualte the state derivative
    xdot = zeros(3,1);
    xdot(1) = sigma;
    xdot(2) = gamma;
    xdot(3) = u;
end

function calculateSymbolicUncontrolled
    syms t tau umax real
    
    % Uncontrolled portion of x_k (after tau)
    x_k_tau = kappaZeroState(tau, umax);
    e_At_tau = [1 t-tau (t-tau)^2/2; 0 1 t-tau; 0 0 1];
    x_k_uncontrolled_calc = e_At_tau * x_k_tau
    
    x_k_uncontrolled_hand = [tau^3/6 + (t-tau)*tau^2/2 + tau*(t-tau)^2/2; tau^2/2 + (t-tau)*tau; tau]*umax;
    x_k_uncont_error = simplify(x_k_uncontrolled_calc - x_k_uncontrolled_hand);
    
    % controlled portion of x_k (after tau)
    x_k_controlled = [(t-tau)^3/6; (t-tau)^2/2; t-tau]*-umax;
    
    
    % Evaluate kappa at 2 tau
    x_k = x_k_uncontrolled_hand + x_k_controlled;
    t = 2*tau;
    x_k = subs(x_k)
    
end

