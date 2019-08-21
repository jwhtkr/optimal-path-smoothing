classdef Unicycle2 < CostClass
        
    properties
        % Cost weights
        p1 = .1; % weight on velocity
        p2 = 0.00000000001; % weight on angular velocity
        p3 = 4; % weight of avoidance
        p5 = .1; % weight on go to goal
        
        % Operational flags
        numericalPartialLogic = false;
        useEulerIntegration = true;
        
        % Cost variables
        qd = []; % Desired position
        qb = []; % Obstacles
        % qb = [[3;3], [4;4]];
        n_obs;
        sig = 10;
        vd = 1; % Desired Velocity
        
        dmin = 0.1; %.25; % Distance from obstacle just before collision
        dmax = 1;%1.25;
        log_dmax_dmin;
        
        % Time Variables
        time_collision;     % time of collison
        collision_detection = false;    % collision bool
        T = 5; % Time variable for integration
        tf = 5;% Final time
        dt = 0.1; % Integration stepsize
        t_span % Stores the span for integration
        t_span_rev % Same as t_span but in reverse order
        t_len % Number of time variables        
        
        k_vel_ctrl;
        
        % Define the control variables
        ind_a1 = 1;
        ind_alpha1 = 2;
        ind_time1 = 3;
        ind_a2 = 4;
        ind_alpha2 = 5;
        ind_time2 = 6;
        ind_a3 = 7;
        ind_alpha3 = 8;
        
        % Define the state indices
        ind_x = 1;
        ind_y = 2;
        ind_q = [1; 2];
        ind_theta = 3; 
        ind_v = 4;
        ind_w = 5;
        ind_cost = 6;       % Used on the aggregate state when the cost is appended to the end
        n_agg = 4;          % Number of aggregate states when cost and dynamics are considered
        
        % Plotting variables
        traj_arc1 = [];
        traj_arc2 = [];
        traj_arc3 = [];
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
            
            obj.cost(u);
            step = @(u) obj.armijo_step(u);
            u = obj.initialize(u);
            while ~obj.armijo_stop(u) %i < 11
                u = u + step(u);             
            end
            u;
            obj.plotTraj(u);
            pause(0.3);
            if u(obj.ind_time1) - obj.dt < 0
                u(obj.ind_time1) = obj.T/3;
                u(obj.ind_time2) = obj.T*2/3;
            else
                u(obj.ind_time1) = u(obj.ind_time1) - obj.dt;
                u(obj.ind_time2) = u(obj.ind_time2) - obj.dt;
            end
            
        end
        
        function u0 = initialize(obj,u)
            min_cost = obj.cost(u);
            u0 = u;
            cost = @(var) obj.cost(var);
            
            w1 = -100/obj.T*3*pi/180:100/obj.T*3*pi/180/15:100/obj.T*3*pi/180;
            
            for i = 1:length(w1)
                u_var = [obj.vd; w1(i);obj.T/3;obj.vd;w1(i);obj.T*2/3;obj.vd;w1(i)];
                %                 obj.plotTraj(u_var);
                %                 pause(0.02);
                if min_cost > cost(u_var)
                    u0 = u_var;
                    min_cost = cost(u_var);
                end
            end
            if isinf(min_cost)
                obj.collision_detection = true;
                obj.cost(u0);
                
                obj.time_collision
                
                if ~obj.collision_detection
                    s = 0.2*obj.time_collision/obj.T;
                    u0(obj.ind_a1) = u0(obj.ind_a1)*s;
                    u0(obj.ind_alpha1) = u0(obj.ind_alpha1)*s;
                    u0(obj.ind_a2) = u0(obj.ind_a2)*s;
                    u0(obj.ind_alpha2) = u0(obj.ind_alpha2)*s;
                    u0(obj.ind_a3) = u0(obj.ind_a3)*s;
                    u0(obj.ind_alpha3) = u0(obj.ind_alpha3)*s;
                    u0(obj.ind_time1) = u0(obj.ind_time1)*s;
                    u0(obj.ind_time2) = u0(obj.ind_time2)*s;
                    obj.T = obj.T*s;
                    obj.cost(u0);
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
            L = obj.p_global(x)*obj.p1/2*(v-obj.vd)^2 + obj.p_global(x)*obj.p2/2*w^2;
            
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
                    L = L + inf;
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
            phi = obj.p_global(x)*obj.p5/2*(err'*err);
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
            x_sol = obj.integrate(@(t,x)obj.unicycleDynamics(t, x, u), obj.x0, true, false);
            
            % Create costate initial conditions
            xT = obj.evalIntResult(x_sol, obj.T);
            lamT = obj.terminalStatePartial(xT)';
            xiT = zeros(2,1);
            costateT = [lamT; xiT];
            
            
            % Simulate costates backward in time Tf to tau2
            obj.setTimeSpan(u,3);

            costates_T = obj.integrate(@(t,costate)obj.costateDynamics(t, costate, x_sol, u), costateT, false, false);
            
            % Evaluate Results, extract xi values and reset for next time
            % interval
            costate2 = obj.evalIntResult(costates_T, u(obj.ind_time2));
            xi2 = costate2(obj.n+1:end);
            lam2 = costate2(1:obj.n);
            costate2(4:end) = 0;
            
            % Simulate costates backwards in time tau2 to tau1
            obj.setTimeSpan(u,2);
            costates_2 = obj.integrate(@(t,costate)obj.costateDynamics(t, costate, x_sol, u), costate2, false, false);
            
            % Evaluate Results, extract xi values and reset for next time
            % interval
            
            costate1 = obj.evalIntResult(costates_2, u(obj.ind_time1));
            xi1 = costate1(obj.n+1:end);
            lam1 = costate1(1:obj.n);
            costate1(4:end) = 0;
            
            % Simulate costates backwards in time tau1 to 0
            obj.setTimeSpan(u,1);
            costates_1 = obj.integrate(@(t,costate)obj.costateDynamics(t, costate, x_sol, u), costate1, false, false);
            
            % Evaluate results, extract xi values and reset time span to 0
            % to T
            costate0 = obj.evalIntResult(costates_1, 0);
            xi0 = costate0(obj.n+1:end);
%             lam0 = costate0(1:obj.n);
            obj.setTimeSpan(u,0);
            
            
            z0 = [obj.x0; 0];
            cost = obj.integrate(@(t,z)obj.costAndStateDynamics(t,z,u), z0, true, true);
            cost0 = obj.evalIntResult(cost,0);
            cost1 = obj.evalIntResult(cost,u(3));
            cost2 = obj.evalIntResult(cost,u(6));
            
            dJ_dtau1 = cost0(obj.ind_cost) - cost1(obj.ind_cost) + lam1'*(cost0(1:obj.n) - cost1(1:obj.n));
            dJ_dtau2 = cost1(obj.ind_cost) - cost2(obj.ind_cost) + lam2'*(cost1(1:obj.n) - cost2(1:obj.n));
            
%             if u(obj.ind_time1) - dJ_dtau1 < 0 || u(obj.ind_time1) - dJ_dtau1 > u(obj.ind_time2) - dJ_dtau2
%                 dJ_dtau1 = 0;
%             end
%             if u(obj.ind_time2) - dJ_dtau2 < u(obj.ind_time1) - dJ_dtau1 || u(obj.ind_time2) - dJ_dtau2 > obj.T
%                 dJ_dtau2 = 0;
%             end
            % Final Partial
            dJ_du = [xi0' dJ_dtau1 xi1' dJ_dtau2 xi2'];
        end
        
        function dphi_dx = terminalStatePartial(obj, x)
            % Extract states and velocities
            q = obj.getPosition(x);
            
            % Calculate partial
            dphi_dx = obj.p_global(x)*obj.p5.*[(q-obj.qd)', 0, 0, 0];
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
            
            dL_dx = dL_dx + obj.p_global(x).*[0 0 0 obj.p1*(v-obj.vd) obj.p2*w];
            
        end
        
        function dL_du = instInputPartial(obj, t, x, u)
            % Extract states and velocities
            [v, w] = obj.getVelocities(x);
            
            dL_du = [0 0]; %[obj.p1*(v-obj.vd), obj.p2*w];
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
%             df_du = [cos(theta), 0; sin(theta), 0; 0 1; 0 0; 0 0];
            df_du = [0, 0; 0, 0; 0 0; obj.k_vel_ctrl];
        end
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%% State and Dynamics functions %%%%
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function [a, alpha] = getAccelerations(obj,u,t)
            if t < u(obj.ind_time1)
                a = u(obj.ind_a1);
                alpha = u(obj.ind_alpha1);
            elseif t >= u(obj.ind_time1) && t < u(obj.ind_time2)
                a = u(obj.ind_a2);
                alpha = u(obj.ind_alpha2);
            else
                a = u(obj.ind_a3);
                alpha = u(obj.ind_alpha3);
            end
            
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
            [a, alpha] = obj.getAccelerations(u,t);
            theta = x(obj.ind_theta);
            
            % Calculate derivative
            xdot = zeros(obj.n, 1);
            xdot(obj.ind_x) = v*cos(theta);
            xdot(obj.ind_y) = v*sin(theta);
            xdot(obj.ind_theta) = w;
            xdot(obj.ind_v) = -obj.k_vel_ctrl(1,1)*(v-a);
            xdot(obj.ind_w) = -obj.k_vel_ctrl(2,2)*(w-alpha);
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
           xvec = obj.integrate(@(t,x)obj.unicycleDynamics(t, x, u), obj.x0, true, false);
%            [tvec, xvec] = ode45(@(t,x)obj.unicycleDynamics(t,x,u), [0 obj.T], obj.x0);
%            xvec = xvec';
           tvec = obj.t_span;
           j = 1;
           k = 1;
           xarc2 = [];
           for i = 1:length(tvec)
               if tvec(i) < u(obj.ind_time1)
                   xarc1(:,i) = xvec(:,i);
               elseif tvec(i) >= u(obj.ind_time1) && tvec(i) <= u(obj.ind_time2)
                   xarc2(:,j) = xvec(:,i);
                   j = j + 1;
               elseif tvec(i) > u(obj.ind_time2) && tvec(i) <= obj.T
                   xarc3(:,k) = xvec(:,i);
                   k = k + 1;
               end
           end
           
           % Plot the data 
           if isempty(obj.traj_arc1)
%                figure;
               obj.traj_arc1 = plot(xarc1(1,:), xarc1(2,:), 'g', 'linewidth', 1);
               obj.traj_arc2 = plot(xarc2(1,:), xarc2(2,:), 'r', 'linewidth', 1);
               obj.traj_arc3 = plot(xarc3(1,:), xarc3(2,:), 'b', 'linewidth', 1);
           else
               set(obj.traj_arc1, 'xdata', xarc1(1,:), 'ydata', xarc1(2,:));
               if ~isempty(xarc2)
                   set(obj.traj_arc2, 'xdata', xarc2(1,:), 'ydata', xarc2(2,:));
               end
               set(obj.traj_arc3, 'xdata', xarc3(1,:), 'ydata', xarc3(2,:));
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
                if isinf(zvec(end,k)) && obj.collision_detection
                    obj.time_collision = obj.t_span(k);
                    obj.collision_detection = false;
                end
            end
            
            % Reverse ordering of zvec if reverse integration performed
            if ~forward
                zvec = zvec(:,[obj.t_len:-1:1]);
            end            
        end
        
        function z = evalIntResult(obj, z_sol, t)
            if obj.useEulerIntegration
                ind = find(obj.t_span == t); 
                if isempty(ind)
                    ind = round(t/obj.dt)+1;
                end
                z = z_sol(:, ind);

            else
                z = deval(z_sol, t);
            end
        end
        
        function setTimeSpan(obj, u, logic)
            switch logic
                case 1
                    obj.t_span = 0:obj.dt:u(obj.ind_time1);
                    obj.t_span_rev = u(obj.ind_time1):-obj.dt:0;
                    obj.t_len = length(obj.t_span);
                case 2
                    obj.t_span = u(obj.ind_time1):obj.dt:u(obj.ind_time2);
                    obj.t_span_rev = u(obj.ind_time2):-obj.dt:u(obj.ind_time1);
                    obj.t_len = length(obj.t_span);
                case 3
                    obj.t_span = u(obj.ind_time2):obj.dt:obj.T;
                    obj.t_span_rev = obj.T:-obj.dt:u(obj.ind_time2);
                    obj.t_len = length(obj.t_span);
                case 0
                    obj.t_span = 0:obj.dt:obj.T;
                    obj.t_span_rev = obj.T:-obj.dt:0;
                    obj.t_len = length(obj.t_span);
            end
        end
        
        function p = p_global(obj,x)
            q = obj.getPosition(x);
%             d = norm(q - obj.qd);
%             p = 1 - exp(-d^2/obj.sig^2);
            p = 1;
        end
        
        function setGoal(obj, q)
            obj.qd = q;
        end
        
        function xT = getTerminalPosition(obj, x0, u)

            %z_sol = ode45(@(t,z)obj.costAndStateDynamics(t,z,u), [0:obj.dt:obj.T], z0, opts);
            xvec = obj.integrate(@(t,x)obj.unicycleDynamics(t, x, u), x0, true, false);
            
            % Extract the final value for the cost
            xT = obj.evalIntResult(xvec, obj.T);
        end
        
        
    end
end

