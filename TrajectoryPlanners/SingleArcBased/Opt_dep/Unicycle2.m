classdef Unicycle2 < CostClass
        
    properties
        % Cost weights
        p1 = 1; % weight on velocity
        p2 = .1; % weight on angular velocity
        p3 = 3; % weight of avoidance
        p5 = .1; % weight on go to goal
        
        % Operational flags
        numericalPartialLogic = false;
        useEulerIntegration = true;
        
        % Cost variables
        qd = [6; 5]; % Desired position
        qb = []; % Obstacles
%         qb = [[4;4]];
        n_obs;
        sig = 10;
        vd = 1; % Desired Velocity
        
        dmin = .25; % Distance from obstacle just before collision
        dmax = 1.1;
        log_dmax_dmin;
        
        
        T = 5; % Final time
        dt = 0.1; % Integration stepsize
        t_span % Stores the span for integration
        t_span_rev % Same as t_space but in reverse order
        t_len % Number of time variables        
        
        % Define the control variables
        ind_a = 1;
        ind_alpha = 2;
        
        k_vel_ctrl;
        
        % Define the state indices
        ind_x = 1;
        ind_y = 2;
        ind_theta = 3; 
        ind_v = 4;
        ind_w = 5;
        ind_cost = 6;       % Used on the aggregate state when the cost is appended to the end
        n_agg = 4;          % Number of aggregate states when cost and dynamics are considered
        
        % Plotting variables
        state_handle = [];
        state_ax = []; % Handle for the state axes
    end
    
    methods
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%%  Initialization functions  %%%
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function obj = Unicycle2(x0)
            % Define limits
%             x1 = [0 1.25];
%             x2 = [-.2 .2];
            x1 = [0 .25];
            x2 = [-.05 .05];
            z = [-10 100];            
            
            % Create the class
            obj = obj@CostClass(x1, x2, z, x0);
            obj.n_obs = size(obj.qb, 2);
            obj.log_dmax_dmin = log(obj.dmax - obj.dmin);
            
            % Control Gain
            A = zeros(2);
            B = eye(2);
            Q = eye(2);
            R = eye(2);
            obj.k_vel_ctrl = lqr(A,B,Q,R);
            
            % Create the timing variables
            obj.t_span = 0:obj.dt:obj.T;
            obj.t_span_rev = obj.T:-obj.dt:0;
            obj.t_len = length(obj.t_span);
                      
        end
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%%  Functions for Interfacing with Sim  %%%
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function reset(obj,x)
            obj.x0 = x;
            obj.iter_num = 1; % stores the number of times the gradient has been called
            obj.n_obs = size(obj.qb, 2);
            
            % Armijo variables
            obj.alpha = 0.5;
            obj.beta = 0.5;
            obj.k = 10;
        end
        
        function setObstacles(obj,xo,yo,d)
           obj.qb = [];
           for i = 1:length(xo)
               if ~isinf(d(i)) %d(i) <= obj.dmax
                   obj.qb =  [obj.qb [xo(i);yo(i)]];
               end
           end
        end
        
        function u = minimize(obj,x,u)
            obj.reset(x);
            
            step = @(u) obj.armijo_step(u);
            u = obj.initialize(u);
            while ~obj.armijo_stop(u) 
                u = u + step(u);
            end
            obj.plotTraj(u);
            pause(0.02);
        end
        
        function u0 = initialize(obj,u)
            min_cost = obj.cost(u);
            u0 = u;
            cost = @(var) obj.cost(var);
            
            % -90 degree to 90 degree turn based on time horizon
            
            
            w = -120/obj.T*pi/180:120/obj.T*pi/180/30:120/obj.T*pi/180;            
            for i = 1:length(w)
                u_var = [obj.vd; w(i)];
%                 obj.plotTraj(u_var);
%                 pause(0.01);
                if min_cost > cost(u_var)
                    u0 = u_var;
                    min_cost = cost(u_var);
                    
                end
            end
            
        end
        
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%%  Functions for Calculating cost  %%%
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        function val = cost(obj,u)
            %cost: This function will simulate both the state vector and and instantaneous cost forward in time
            
            % Create aggregate initial state
            z0 = [obj.x0; 0];
            %opts = odeset('MaxStep',obj.dt);
            %z_sol = ode45(@(t,z)obj.costAndStateDynamics(t,z,u), [0:obj.dt:obj.T], z0, opts);
            z_sol = obj.integrate(@(t,z)obj.costAndStateDynamics(t,z,u), z0, true, true);
            
            % Extract the final value for the cost
            zT = obj.evalIntResult(z_sol, obj.T);
            xT = zT(1:obj.n);
            L = zT(obj.ind_cost);
            
            % Add the terminal cost
            val = L + obj.terminalCost(xT, u);
        end
        
        function L = instantaneousCost(obj, t, x, u)
            % Extract states and velocities
            [v, w] = obj.getVelocities(x);
            q = obj.getPosition(x);
            
            % Get portion for velocities
            L = obj.p1/2*(v-obj.vd)^2 + obj.p2/2*w^2;
            
            % Get portion for obstacles
            for k = 1:obj.n_obs
                % Extract obstacle of interest
                q_k = obj.qb(:,k);
                
                % Calculate distance to obstacle
                d = norm(q - q_k);
                
                % Calculate contribution based on distance variables
                if d > obj.dmin && d < obj.dmax
                    Lavoid = obj.p3*(obj.log_dmax_dmin - log(d - obj.dmin));
                    L = L + Lavoid;
                elseif d < obj.dmin
                    L = L + obj.p3*inf;
                    if isnan(L)
                        L = 0;
                    end
                end
            end
            
        end
        
        function phi = terminalCost(obj, x, u)
            % Extract position
            q = obj.getPosition(x);
            
            % Calculate cost as squared distance to goal
            err = q - obj.qd;
            phi = obj.p5/2*(err'*err);
        end
        
        function zdot = costAndStateDynamics(obj, t, z, u)
            % Extract state and cost
            x = z(1:obj.n);
            %L = z(obj.ind_cost);
            
            % Calculate time derivative of each component
            xdot = obj.unicycleDynamics(t, x, u);
            Ldot = obj.instantaneousCost(t, x, u);
            
            % Update state
            zdot = zeros(obj.n_agg, 1);
            zdot(1:obj.n) = xdot;
            zdot(obj.ind_cost) = Ldot;
        end
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%% Function for calculating partial %%%
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        function dJ_du = partial(obj, u)
            if obj.numericalPartialLogic
                dJ_du = obj.numericPartial(u);
                return;
            end      
           
            % Simulate state forward in time
            %x_sol = ode45(@(t,x)obj.unicycleDynamics(t, x, u), [0:obj.dt:obj.T], obj.x0);
            x_sol = obj.integrate(@(t,x)obj.unicycleDynamics(t, x, u), obj.x0, true, false);
            
            
            % Create costate initial conditions
            xT = obj.evalIntResult(x_sol, obj.T);
            lamT = obj.terminalStatePartial(xT)';
            xiT = zeros(2,1);
            costateT = [lamT; xiT];
            
            % Simulate costates backward in time
            %costates = ode45(@(t,costate)obj.costateDynamics(t, costate, x_sol, u), [obj.T:-obj.dt:0], costateT); 
            costates = obj.integrate(@(t,costate)obj.costateDynamics(t, costate, x_sol, u), costateT, false, false);
            
            % Calculate partial
            costate0 = obj.evalIntResult(costates, 0);
            xi0 = costate0(obj.n+1:end);
            dJ_du = xi0';
        end
        
        function dphi_dx = terminalStatePartial(obj, x)
            % Extract states and velocities
            q = obj.getPosition(x);
            
            % Calculate partial
            dphi_dx = obj.p5.*[(q-obj.qd)', 0, 0, 0];
        end
            
        function dL_dx = instStatePartial(obj, t, x, u)
            % Extract states
            [v, w] = obj.getVelocities(x);
            q = obj.getPosition(x);
            
            % Calculate necesssary derivatives
            dL_dx = zeros(1, obj.n);
            dq_dx = [eye(2), [0; 0], [0; 0], [0; 0]];
            
            % Loop through obstacles to get contribution of each obstacle
            for k = 1:obj.n_obs
                % Extract the obstacle point
                q_k = obj.qb(:,k);
                
                % Calculate distance to the obstacle
                d = norm(q-q_k);
                
                % If within the range, then calculate the portion
                % corresponding to the obstacle
                dLavoid_dd = 0;
                if d > obj.dmin && d < obj.dmax
                    dLavoid_dd = -obj.p3/(d-obj.dmin);
                elseif d < obj.dmin
                    dLavoid_dd = -inf;
                end
                
                % Calculate partial of distance with respect to position
                dd_dq = (q - q_k)' / d;
                
                % Calculate contribution of avoidance
                dLavoid_dx = dLavoid_dd * dd_dq * dq_dx;
                dL_dx = dL_dx + dLavoid_dx;
            end
            
            dL_dx = dL_dx + [0 0 0 obj.p1*(v-obj.vd) obj.p2*w];
            
        end
        
        function dL_du = instInputPartial(obj, t, x, u)
            % Extract states and velocities
            [v, w] = obj.getVelocities(x);
            
            dL_du = [0 0];
%             dL_du = [obj.p1*(v-obj.vd), obj.p2*w];
        end
        
        function df_dx = dynamicsStatePartial(obj, t, x, u)
            % Get velocities and state
            [v, w] = getVelocities(obj, x);
            theta = x(obj.ind_theta);
            
            % Calculate partial
            df_dx = [0 0 -v*sin(theta) cos(theta) 0; 0 0 v*cos(theta) sin(theta) 0; 0 0 0 0 1; 0 0 0 0 0; 0 0 0 0 0];
        end
        
        function df_du = dynamicsInputPartial(obj, t, x, u)
            % Get orientation
            theta = x(obj.ind_theta);
            
            % Calculate partial
            %df_du = [cos(theta), 0; sin(theta), 0; 0 1];
            df_du = [0, 0; 0, 0; 0 0; obj.k_vel_ctrl];
        end
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%% State and Dynamics functions %%%%
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function [a, alpha] = getAccelerations(obj,u)
            a = u(obj.ind_a);
            alpha = u(obj.ind_alpha);
        end
            
        function [v, w] = getVelocities(obj, x)
            v = x(obj.ind_v);
            w = x(obj.ind_w);
        end
        
        function q = getPosition(obj, x)
            q = [x(obj.ind_x); x(obj.ind_y)];
        end
        
        function xdot = unicycleDynamics(obj, t, x, u)
            % Extract states and velocities
            [v, w] = obj.getVelocities(x);
%             [a, alpha] = obj.getAccelerations(u);
            theta = x(obj.ind_theta);
            
            % Calculate derivative
            xdot = zeros(obj.n, 1);
            xdot(obj.ind_x) = v*cos(theta);
            xdot(obj.ind_y) = v*sin(theta);
            xdot(obj.ind_theta) = w;
            xdot(obj.ind_v) = -obj.k_vel_ctrl(1,1)*(v-u(1));
            xdot(obj.ind_w) = -obj.k_vel_ctrl(2,2)*(w-u(2));
        end
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%% Costate Dynamics functions %%%%
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function costate_dot = costateDynamics(obj, t, costate, x_sol, u)
            % Extract state
            x = obj.evalIntResult(x_sol, t);
            
            % Extract costates
            lam = costate(1:obj.n);
            
            % Calculate dynamics
            lam_dot = obj.lamDynamics(t, lam, x, u);
            xi_dot = obj.xiDynamics(t, lam, x, u);
            
            % Aggregate dynamics
            costate_dot = [lam_dot; xi_dot];            
        end
        
        function lam_dot = lamDynamics(obj, t, lam, x, u)
            % Calculate necessary partials
            dL_dx = obj.instStatePartial(t, x, u);
            df_dx = obj.dynamicsStatePartial(t, x, u);
            
            % Calculate dynamics
            lam_dot = -dL_dx' - df_dx'*lam;
        end
        
        function xi_dot = xiDynamics(obj, t, lam, x, u)
            % Get necessary partials
            dL_du = obj.instInputPartial(t, x, u);
            df_du = obj.dynamicsInputPartial(t, x, u);
            
            % Calculate dynamics
            xi_dot = -dL_du' - df_du'*lam;
        end
        
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%% Plotting functions %%%%
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function plotCost(obj, varargin)
           
            % Calculate values to be evaluated
            x1_vec = [obj.x1lim(1):obj.dx:obj.x1lim(2)];
            x2_vec = [obj.x2lim(1):obj.dx:obj.x2lim(2)];
            [X1, X2] = meshgrid(x1_vec, x2_vec);
            
            % Get cost function to be used
            if nargin < 2
                costfunction = @(x)obj.cost(x);
            else
                costfunction = @(x)varargin{1}(x);
            end
            
            % Loop through and evaluate each value
            Z = zeros(size(X1));
            for m = 1:size(X1,1)
                for n = 1:size(X1,2)
                    x = [X1(m,n); X2(m,n)];
                    c = costfunction(x);
%                     if c == inf || isnan(c)
%                         display('inf cost at ',x)
%                     end
                    Z(m,n) = min(obj.zlim(2), c);
                end
            end
            
            % Plot the surf and contour plots
            if isempty(obj.surf_contour_fig)
                obj.surf_contour_fig = figure;
            end
            subplot(1,2,1);
            surf(X1,X2,Z)
            zlim(obj.zlim);            
            colormap(pink);
            shading interp
            
            subplot(1,2,2);
            contour(X1,X2, Z, 70);
%             contour(X1,X2, Z, [.25:.01:.75, .75:1:50]);
            hold on;
        end
        
        function plotState(obj, x)
            if obj.n ~= 5 % Only plot for 2D optimization
                return;
            end
            
            set(0, 'currentfigure', obj.surf_contour_fig);
            subplot(1,2,2);
            plot(x(1), x(2), 'ko', 'linewidth', 3); 
%             xlim(obj.x1lim);
%             ylim(obj.x2lim);
%             axis equal
        end
        
        function plotTraj(obj, u)
           % Calculate the state trajectory
           [tvec, xvec] = ode45(@(t,x)obj.unicycleDynamics(t,x,u), [0 obj.T], obj.x0);
           xvec = xvec';
           
           % Plot the data 
           if isempty(obj.state_handle)
%                figure;
               obj.state_handle = plot(xvec(1,:), xvec(2,:), 'g', 'linewidth', 1); %hold on;
%                for i = 1:length(obj.qb(1,:))
%                    plot(obj.qb(1,i), obj.qb(2,i), 'ko', 'linewidth', 2);
%                    circle(obj.qb(:,i), obj.dmin, 20, 'r', []);
%                    circle(obj.qb(:,i), obj.dmax, 50, 'b', []);
%                end
%                plot(obj.qd(1), obj.qd(2), 'go', 'linewidth', 4); % Plot the goal position
% %               ylim([0 max(xvec(2,:))]);
%                obj.state_ax = gca;
%                title('Trajectory of unicycle');
           else
               set(obj.state_handle, 'xdata', xvec(1,:), 'ydata', xvec(2,:));
 %              set(obj.state_ax, 'ylim', [0 max(xvec(2,:))]);
           end
        end
        
        function step = optimal_step(obj, x)
            step = [0;0];
        end
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%% Integration functions %%%%
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function z_sol = integrate(obj, dynamics, z0, forward, force_max_step)
            if obj.useEulerIntegration
                z_sol = obj.eulerIntegration(dynamics, z0, forward);
            else
                if force_max_step
                    opts = odeset('MaxStep',obj.dt);
                else
                    opts = [];
                end
                
                if forward
                    z_sol = ode45(dynamics, obj.t_span, z0, opts);
                else
                    z_sol = ode45(dynamics, obj.t_span_rev, z0, opts);
                end
            end
        end
        
        function zvec = eulerIntegration(obj, dynamics, z0, forward)
            % Determine if moving forward or backward
            if forward
                t = @(k) obj.t_span(k);
                del_t = obj.dt;
            else
                t = @(k) obj.t_span_rev(k);
                del_t = -obj.dt;
            end
            
            % Create initial z values
            zvec = zeros(length(z0), obj.t_len);
            z_act = z0;
            zvec(:,1) = z_act;
            
            % Perform integration            
            for k = 2:obj.t_len                
                z_act = z_act + dynamics(t(k), z_act)*del_t;
                zvec(:,k) = z_act;
            end
            
            % Reverse ordering of zvec if reverse integration performed
            if ~forward
                zvec = zvec(:,[obj.t_len:-1:1]);
            end            
        end
        
        function z = evalIntResult(obj, z_sol, t)
            if obj.useEulerIntegration
                ind = floor(t/obj.dt*100/100)+1;%round(t/obj.dt)+1;
                z = z_sol(:, ind);
            else
                z = deval(z_sol, t);
            end
        end
        
        function setTimeSpan(obj)
            obj.t_span = 0:obj.dt:obj.T;
            obj.t_span_rev = obj.T:-obj.dt:0;
            obj.t_len = length(obj.t_span);
        end
        
        
    end
end

