classdef SmoothCurvature < handle
    % curvature parameters
    properties (SetAccess=protected)
        umax % maximum third derivative of curvature
        sig_max % maximum first derivative of curvature
        kappa_max % maximum curvature
    end
    
    % curvature switch times
    properties (SetAccess=protected)
        tau % switch time for bang-bang control
        t_0 % Initial time
        t_accel_2_decel % Time to switch from full acceleration to deceleration
        t_sig_max % Time to switch to no input
        t_sigma_const_final % Final time for executing sig_max
        t_decel_2_accel % Time to switch from maximum deceleration to maximum acceleration of sigma
        t_kappa_max % time that kappa_max is achieved
    end
    
    % States at switch times
    properties (SetAccess=protected)
        x_0 = zeros(3,1); % Initial state
        x_accel_2_decel % State when switch from full acceleration to deceleration
        x_sig_max % State when switch to no input
        x_sigma_const_final % State when for executing sig_max
        x_decel_2_accel % State when switch from maximum deceleration to maximum acceleration of sigma
        x_kappa_max % State when kappa_max is achieved            
    end
    
    methods
        function obj = SmoothCurvature(umax, sig_max, kappa_max,varargin)
            %computeClothoidCurvature: Create instance of this class
            %
            % Inputs:
            %   umax: maximum third derivative of curvature
            %   sig_max: maximum first derivative of curvature
            %   kappa_max: maximum curvature
            
            % Store input variables
            obj.umax = umax;
            obj.sig_max = sig_max;
            obj.kappa_max = kappa_max;
            
            % Compute time value for switching on bang-bang control
            obj.tau = sqrt(sig_max/umax); % switch time

            % Compute time values for achieving kappa_max
            delta_kappa = umax*(sig_max/umax)^(3/2); % Change in curvature when converging to sig_max
            delta_kappa_const = kappa_max - 2*delta_kappa; % Change in curvature operating at sig_max
            delta_t_sigma_const = delta_kappa_const/sig_max; % Time executing maximum sig_max

            % Check for malformed problem
            if delta_kappa_const < 0
                error('umax too slow to reach constant sigma value');
            end    

            % Calculate the time at each key point
            obj.t_0 = 0; % Initial time
            obj.t_accel_2_decel = obj.tau; % Time to switch from full acceleration to deceleration
            obj.t_sig_max = 2*obj.tau; % Time to switch to no input
            obj.t_sigma_const_final = delta_t_sigma_const + obj.t_sig_max; % Final time for executing sig_max
            obj.t_decel_2_accel = obj.t_sigma_const_final + obj.tau; % Time to switch from maximum deceleration to maximum acceleration of sigma
            obj.t_kappa_max = obj.t_decel_2_accel + obj.tau; % time that kappa_max is achieved

            t_switch = [
                obj.t_0;...
                obj.t_accel_2_decel;...
                obj.t_sig_max;...
                obj.t_sigma_const_final;...
                obj.t_decel_2_accel;...
                obj.t_kappa_max];
            
%             t_switch = flip(t_switch);
            
            % Calculate state at each key point
            if ~isempty(varargin) > 0
                obj.x_0  = varargin{1}; 
                direction = varargin{2};
                obj.x_accel_2_decel = kappaStateConstControl(t_switch(2), t_switch(1), obj.x_0, direction*-umax); % State when switch from full acceleration to deceleration
                obj.x_sig_max = kappaStateConstControl(t_switch(3), t_switch(2), obj.x_accel_2_decel, direction*umax); % State when switch to no input
                obj.x_sigma_const_final = kappaStateConstSigma(t_switch(4), t_switch(3), obj.x_sig_max(1), direction*-sig_max); % State when for executing sig_max
                obj.x_decel_2_accel = kappaStateConstControl(t_switch(5), t_switch(4), obj.x_sigma_const_final, direction*umax); % State when switch from maximum deceleration to maximum acceleration of sigma
                obj.x_kappa_max = kappaStateConstControl(t_switch(6), t_switch(5), obj.x_decel_2_accel, direction*-umax); % State when kappa_max is achieved  
            else
                obj.x_0 = zeros(3,1); % Initial state
                obj.x_accel_2_decel = kappaStateConstControl(obj.t_accel_2_decel, obj.t_0, obj.x_0, umax); % State when switch from full acceleration to deceleration
                obj.x_sig_max = kappaStateConstControl(obj.t_sig_max, obj.t_accel_2_decel, obj.x_accel_2_decel, -umax); % State when switch to no input
                obj.x_sigma_const_final = kappaStateConstSigma(obj.t_sigma_const_final, obj.t_sig_max, obj.x_sig_max(1), sig_max); % State when for executing sig_max
                obj.x_decel_2_accel = kappaStateConstControl(obj.t_decel_2_accel, obj.t_sigma_const_final, obj.x_sigma_const_final, -umax); % State when switch from maximum deceleration to maximum acceleration of sigma
                obj.x_kappa_max = kappaStateConstControl(obj.t_kappa_max, obj.t_decel_2_accel, obj.x_decel_2_accel, umax); % State when kappa_max is achieved
            end
              
        end
        
        function x_mat = calculateClothoidCurvature(obj, t_vec, varargin)
        %calculateClothoidCurvature returns a vector of kappa states that converge to
        %kappa_max as fast as possible using umax
        %
        % The kappa state is a 3x1 vector:
        %         [kappa]
        %   x_k = [sigma]
        %         [gamma]
        %
        % Inputs:
        %   t_vec: 1xm vector of time values   
        %
        % Outputs:
        %   x_mat: 3xm matrix of kappa states where each column corresponds to the
        %          t_vec time value       
        
        if ~isempty(varargin)
            initialKappaState = varargin{1};
            direction = varargin{2};
        else
            initialKappaState = false;
            direction = 1;
        end

            % Get number of time values
            len = length(t_vec); % number of states

            % Calculate kappa state
            x_mat = zeros(3,len); % Storage for calculated states
            for k = 1:len
                t = t_vec(k);
                x_mat(:,k) = obj.getCurvatureStateVector(t, initialKappaState, direction);
            end
        end
        
        function t_span = getCurvatureTimeSpan(obj, dt)
        %getCurvatureTimeSpan returns a time span that will include the
        %entire curvature spaced at time intervales of dt
            t_span = obj.t_0:dt:obj.t_kappa_max; % time vector            
        end
        
        function x_k = getCurvatureStateVector(obj, t, varargin)
            %calculateClothoidCurvature returns a vector of kappa states
            %for a given time value
            %
            % The kappa state is a 3x1 vector:
            %         [kappa]
            %   x_k = [sigma]
            %         [gamma]
            %
            % Inputs:
            %   t: time value of interest (must be greater than or equal to zero)
            %
            % Outputs:
            %   x_k: curvature state at time t
          
            if ~isempty(varargin)
                initialKappaState = varargin{1};
                direction = varargin{2};
            else
                initialKappaState = false;
                direction = 1;
            end
            
            t_switch = [
                obj.t_0;...
                obj.t_accel_2_decel;...
                obj.t_sig_max;...
                obj.t_sigma_const_final;...
                obj.t_decel_2_accel;...
                obj.t_kappa_max];
            
            
            % Calculate the curvature state
            if initialKappaState
%                 t_switch = flip(t_switch);
                if t < t_switch(1)
                    x_k = zeros(3,1);
                elseif t < t_switch(2)
                    x_k = kappaStateConstControl(t, t_switch(1), obj.x_0, direction*-obj.umax);
                elseif t < t_switch(3)
                    x_k = kappaStateConstControl(t, t_switch(2), obj.x_accel_2_decel, direction*obj.umax);
                elseif t < t_switch(4)
                    x_k = kappaStateConstSigma(t, t_switch(3), obj.x_sig_max(1), direction*-obj.sig_max);
                elseif t < t_switch(5)
                    x_k = kappaStateConstControl(t, t_switch(4), obj.x_sigma_const_final, direction*obj.umax);
                else
                    x_k = kappaStateConstControl(t, t_switch(5), obj.x_decel_2_accel, direction*-obj.umax);
                end
            else
                if t < obj.t_0
                    x_k = zeros(3,1);
                elseif t < obj.t_accel_2_decel
                    x_k = kappaStateConstControl(t, obj.t_0, obj.x_0, obj.umax);
                elseif t < obj.t_sig_max
                    x_k = kappaStateConstControl(t, obj.t_accel_2_decel, obj.x_accel_2_decel, -obj.umax);
                elseif t < obj.t_sigma_const_final
                    x_k = kappaStateConstSigma(t, obj.t_sig_max, obj.x_sig_max(1), obj.sig_max);
                elseif t < obj.t_decel_2_accel
                    x_k = kappaStateConstControl(t, obj.t_sigma_const_final, obj.x_sigma_const_final, -obj.umax);
                else
                    x_k = kappaStateConstControl(t, obj.t_decel_2_accel, obj.x_decel_2_accel, obj.umax);
                end
            end
        end
        
        function [kappa, sigma, gamma] = getCurvatureState(obj, t,varargin)
            %calculateClothoidCurvature returns a vector of kappa states
            %for a given time value
            %
            % The kappa state is a 3x1 vector:
            %         [kappa]
            %   x_k = [sigma]
            %         [gamma]
            %
            % Inputs:
            %   t: time value of interest (must be greater than or equal to zero)
            %
            % Outputs:
            %   kappa: curvature at time t
            %   sigma: curvature rate at time t
            %   gamma: curvature acceleration at time t
            if length(varargin) > 0
                initialKappaState = varargin{1};
                direction = varargin{2};
            else
                initialKappaState = false;
                direction = 1;
            end
                
            
            % Calculate the curvature state
            x_k = obj.getCurvatureStateVector(t,initialKappaState, direction);
            
            % Output the curvature state
            kappa = x_k(1);
            sigma = x_k(2);
            gamma = x_k(3);            
        end
        
        function [t_switch, u_switch] = extractSwitchTimesAndControl(obj)
        %extractSwitchTimesAndControl: Returns the times where the third
        %derivative switchings and the corresponding control input
        %
        % Outputs:
        %   t_switch: vector of times when the dynamics switch
        %   u_switch: vector of control values ( u_switch(i) is executed while time < t_switch(i)
        
            t_switch = [
                obj.t_accel_2_decel;...
                obj.t_sig_max;...
                obj.t_sigma_const_final;...
                obj.t_decel_2_accel;...
                obj.t_kappa_max];
            u_switch = [ % execute u_switch(i) if t < t_switch(i)
                obj.umax;...
                -obj.umax;...
                0;...
                -obj.umax;...
                obj.umax];
        end
        
        function u_vec = getControl(obj, t_vec)
        %getControl: outputs the control inputs for a vector of time values
        
            % Initialize the output
            len = length(t_vec);
            u_vec = zeros(1,len);
            
            % Calculate the control
            for k = 1:len
                t = t_vec(k);
                if t < obj.t_0
                    u_vec(k) = 0;
                elseif t < obj.t_accel_2_decel
                    u_vec(k) = obj.umax;
                elseif t < obj.t_sig_max
                    u_vec(k) = -obj.umax;
                elseif t < obj.t_sigma_const_final
                    u_vec(k) = 0;
                elseif t < obj.t_decel_2_accel
                    u_vec(k) = -obj.umax;
                else
                    u_vec(k) = obj.umax;
                end
            end
        end
    end
    
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
%   t: time greater than obj.tau to be evaluated
%   obj.tau: switch time
%   umax: the maximum derivative of gamma
%
% Outputs:
%   x_k: kappa state at time t

    x_k = [t.^3 ./6; t.^2./2; t].*umax;        
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
%sigma max at time obj.t_sig_max
%
% The kappa state is a 3x1 vector:
%         [kappa]
%   x_k = [sigma]
%         [gamma]
%
% Inputs:
%   t: time greater than obj.t_sig_max to be evaluated
%   t_sig_max: time at which sig_max was achieved (and gamma = 0)
%   k_sig_max: the value of kappa at time obj.t_sig_max
%   sig_max: the maximum change in curvature
%
% Outputs:
%   x_k: kappa state at time t

    x_k = [k_sig_max + (t-t_sig_max)*sig_max; ...
           sig_max; ...
           0];
end
