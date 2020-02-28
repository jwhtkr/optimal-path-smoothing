function checkKappaState
close all;

    % Set time values for switching on bang-bang control
    umax = 100; % max control input
    sig_max = .35; % Max value for sigma
    kappa_max = .35; % Maximum desired curvature value
    dt = 0.001;
    
    % Compute the clothoid curvature
    curvature = SmoothCurvature(umax, sig_max, kappa_max);
    t_vec = curvature.getCurvatureTimeSpan(dt);
    x_vec_calc = curvature.calculateClothoidCurvature(t_vec);
    u_vec = curvature.getControl(t_vec);
    [t_switch, u_switch] = curvature.extractSwitchTimesAndControl();
    
    % Simulate forward in time
%     [t_vec_sim, x_vec_sim] = ode45(@(t,x) kappaStateSwitchedDynamics(t, x, t_switch, u_switch), t_vec, zeros(3,1));
%     x_vec_sim = x_vec_sim';
    
    %% Plot the results
    % Create the figure and plot variables
    figure('units','normalized','outerposition',[0 0 1 1]);
    color = 'b';
    col_thresh = 'r:';
    col_switch = 'k:';
    fontsize = 18;
    
    subplot(4,1,1); % Plot the kappa state
    plot(t_vec, x_vec_calc(1,:), color, 'linewidth', 3); hold on;
    plot([t_vec(1) t_vec(end)], [kappa_max kappa_max], col_thresh, 'linewidth', 2);
    plotSwitchTimes(t_switch, col_switch, 2);
    %plot(t_vec_sim, x_vec_sim(1,:), 'b', 'linewidth', 2);
    ylabel('\kappa', 'fontsize', fontsize);
    set(gca, 'fontsize', fontsize);
    
    subplot(4,1,2); % Plot the sigma state
    plot(t_vec, x_vec_calc(2,:), color, 'linewidth', 3); hold on;
    plot([t_vec(1) t_vec(end)], [sig_max sig_max], col_thresh, 'linewidth', 2);
    plotSwitchTimes(t_switch, col_switch, 2);
    %plot(t_vec_sim, x_vec_sim(2,:), 'b', 'linewidth', 2);
    ylabel('\sigma', 'fontsize', fontsize);
    set(gca, 'fontsize', fontsize);
    
    subplot(4,1,3); % Plot the gamma state
    plot(t_vec, x_vec_calc(3,:), color, 'linewidth', 3); hold on;
    plotSwitchTimes(t_switch, col_switch, 2);
    %plot(t_vec_sim, x_vec_sim(3,:), 'b', 'linewidth', 2);
    ylabel('\gamma', 'fontsize', fontsize);
    set(gca, 'fontsize', fontsize);
    
    subplot(4,1,4); % Plot the control input
    plot(t_vec, u_vec, color, 'linewidth', 3); hold on;
    plotSwitchTimes(t_switch, col_switch, 2);
    %plot(t_vec_sim, x_vec_sim(3,:), 'b', 'linewidth', 2);
    ylabel('u', 'fontsize', fontsize);
    set(gca, 'fontsize', fontsize);
    
end

function plotSwitchTimes(t_switch, color, linewidth)
    y_lim = get(gca, 'ylim');
    for k = 1:length(t_switch)
        plot([t_switch(k) t_switch(k)], y_lim, color, 'linewidth', linewidth);
    end
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

