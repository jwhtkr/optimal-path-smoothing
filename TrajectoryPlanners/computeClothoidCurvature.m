function [t_vec, x_mat, t_switch, u_switch] = computeClothoidCurvature(umax, sig_max, kappa_max, dt)
%computeClothoidCurvature returns a vector of kappa states that converge to
%kappa_max as fast as possible using umax
%
% The kappa state is a 3x1 vector:
%         [kappa]
%   x_k = [sigma]
%         [gamma]
%
% Inputs:
%   umax: maximum third derivative of curvature
%   sig_max: maximum first derivative of curvature
%   kappa_max: maximum curvature
%   dt: time step for simulation
%
% Outputs:
%   t_vec: 1xm vector of time values
%   x_mat: 3xm matrix of kappa states where each column corresponds to the
%          t_vec time value
%   t_switch: vector of times when the dynamics switch
%   u_switch: vector of control values ( u_switch(i) is executed while time < t_switch(i)

    % Compute time value for switching on bang-bang control
    tau = sqrt(sig_max/umax); % switch time
    
    % Compute time values for achieving kappa_max
    delta_kappa = umax*(sig_max/umax)^(3/2); % Change in curvature when converging to sig_max
    delta_kappa_const = kappa_max - 2*delta_kappa; % Change in curvature operating at sig_max
    delta_t_sigma_const = delta_kappa_const/sig_max; % Time executing maximum sig_max
    
    % Check for malformed problem
    if delta_kappa_const < 0
        error('umax too slow to reach constant sigma value');
    end    
    
    % Calculate the time at each key point
    t_0 = 0; % Initial time
    t_accel_2_decel = tau; % Time to switch from full acceleration to deceleration
    t_sig_max = 2*tau; % Time to switch to no input
    t_sigma_const_final = delta_t_sigma_const + t_sig_max; % Final time for executing sig_max
    t_decel_2_accel = t_sigma_const_final + tau; % Time to switch from maximum deceleration to maximum acceleration of sigma
    t_kappa_max = t_decel_2_accel + tau; % time that kappa_max is achieved
        
    % Calculate state at each key point
    x_0 = zeros(3,1); % Initial state
    x_accel_2_decel = kappaStateConstControl(t_accel_2_decel, t_0, x_0, umax); % State when switch from full acceleration to deceleration
    x_sig_max = kappaStateConstControl(t_sig_max, t_accel_2_decel, x_accel_2_decel, -umax); % State when switch to no input
    x_sigma_const_final = kappaStateConstSigma(t_sigma_const_final, t_sig_max, x_sig_max(1), sig_max); % State when for executing sig_max
    x_decel_2_accel = kappaStateConstControl(t_decel_2_accel, t_sigma_const_final, x_sigma_const_final, -umax); % State when switch from maximum deceleration to maximum acceleration of sigma
    x_kappa_max = kappaStateConstControl(t_kappa_max, t_decel_2_accel, x_decel_2_accel, umax); % State when kappa_max is achieved    

    % Simulation variables - note: ode45 has a hard time matching when tau
    % is not a multiple of dt
    dt = 0.001; % time spacing 
    tf = t_decel_2_accel+tau; % final time
    t_vec = 0:dt:tf; % time vector
    len = length(t_vec); % number of states
    
    %% Calculate kappa state
    x_mat = zeros(3,len); % Storage for calculated states
    for k = 1:len
        t = t_vec(k);
        if t < t_accel_2_decel
            x_mat(:,k) = kappaStateConstControl(t, t_0, x_0, umax);
        elseif t < t_sig_max
            x_mat(:,k) = kappaStateConstControl(t, t_accel_2_decel, x_accel_2_decel, -umax);
        elseif t < t_sigma_const_final
            x_mat(:,k) = kappaStateConstSigma(t, t_sig_max, x_sig_max(1), sig_max);
        elseif t < t_decel_2_accel
            x_mat(:,k) = kappaStateConstControl(t, t_sigma_const_final, x_sigma_const_final, -umax);
        else
            x_mat(:,k) = kappaStateConstControl(t, t_decel_2_accel, x_decel_2_accel, umax);
        end
    end
    
    %% Output values for switch times
    t_switch = [
        t_accel_2_decel;...
        t_sig_max;...
        t_sigma_const_final;...
        t_decel_2_accel;...
        t_kappa_max];
    u_switch = [ % execute u_switch(i) if t < t_switch(i)
        umax;...
        -umax;...
        0;...
        -umax;...
        umax];
end

function x_k = kappaZeroState(t, umax)
%kappaStateBeforeSwitch returns the kappa state at time t assuming that the
%state started at x_k(0) = 0
%
% The kappa state is a 3x1 vector:
%         [kappa]
%   x_k = [sigma]
%         [gamma]
%
% Inputs:
%   t: time greater than tau to be evaluated
%   tau: switch time
%   umax: the maximum derivative of gamma
%
% Outputs:
%   x_k: kappa state at time t

    x_k = [t.^3 ./6; t.^2./2; t].*umax;        
end

function x_k = kappaStateAfterSwitch(t, tau, umax)
%kappaStateAfterSwitch returns the kappa state at time t having switched from 
% umax to -umax at time tau
%
% The kappa state is a 3x1 vector:
%         [kappa]
%   x_k = [sigma]
%         [gamma]
%
% Inputs:
%   t: time greater than tau to be evaluated
%   tau: switch time
%   umax: the maximum derivative of gamma
%
% Outputs:
%   x_k: kappa state at time t

    % Calculate the value after time tau
    x_k_uncontrolled = [tau^3/6 + (t-tau)*tau^2/2 + tau*(t-tau)^2/2; tau^2/2 + (t-tau)*tau; tau]*umax;
    x_k_zero_state = kappaZeroState(t-tau, -umax);
    x_k = x_k_uncontrolled + x_k_zero_state;
end

function x_k = kappaStateConstControl(t, t_const, xk_const, u_const)
%kappaStateConstControl returns the kappa state at time t having switched to 
% a constant control at time t_const with control u_const and state
% xk_const
%
% The kappa state is a 3x1 vector:
%         [kappa]
%   x_k = [sigma]
%         [gamma]
%
% Inputs:
%   t: time greater than t_const to be evaluated
%   t_const: time at which the state was switched
%   xk_const: state at time t_const
%   u_const: constant control input from t_const to t
%
% Outputs:
%   x_k: kappa state at time t
    
    % Calculate the uncontrolled portion of the state:
    % e^{A*(t-t_const)}*xk_const
    x_uncontrolled = [1, t-t_const, (t-t_const)^2/2; 0, 1, t-t_const; 0 0 1] * xk_const;
    
    % Calculate the controlled portion of the state
    x_controlled = kappaZeroState(t-t_const, u_const);
    
    % Calculate the state
    x_k = x_uncontrolled + x_controlled;    
end

function x_k = kappaStateConstSigma(t, t_sig_max, k_sig_max, sig_max)
%kappaStateConstSigma returns the kappa state at time t having achived
%sigma max at time t_sig_max
%
% The kappa state is a 3x1 vector:
%         [kappa]
%   x_k = [sigma]
%         [gamma]
%
% Inputs:
%   t: time greater than t_sig_max to be evaluated
%   t_sig_max: time at which sig_max was achieved (and gamma = 0)
%   k_sig_max: the value of kappa at time t_sig_max
%   sig_max: the maximum change in curvature
%
% Outputs:
%   x_k: kappa state at time t

    x_k = [k_sig_max + (t-t_sig_max)*sig_max; ...
           sig_max; ...
           0];
end
