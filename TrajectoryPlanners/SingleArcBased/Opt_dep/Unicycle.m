classdef Unicycle < CostClass
        
    properties
        % Cost weights
        p1 = 1; % weight on velocity
        p2 = 1; % weight on angular velocity
        p3 = 0.05; % weight of avoidance
        p5 = 1; % weight on go to goal
        
        numericalPartialLogic = false;
        
        % Cost variables
        qd = [6; 6]; % Desired position
        qb = [[1;1],[2;2],[3;3],[4;4],[5;5],[3;4],[3;2],[2;3],[4;5]]; % Obstacles
%         qb = [[3;3], [4;4]];
        sig = 10;
        vd = 1; % Desired Velocity
        
        dmin = 0.1; % Distance from obstacle just before collision
        dmax = .5;
        
        
        T = 10; % Final time
        dt = 0.1; % Integration stepsize
        
        
        % Plotting variables
        state_handle = [];
        state_ax = []; % Handle for the state axes
    end
    
    methods
        function obj = Unicycle(x0)
            % Define limits
            x1 = [0 1.25];
            x2 = [-.2 .2];
            z = [-10 100];
            
            
            % Create the class
            obj = obj@CostClass(x1, x2, z, x0);
                      
        end
        
        %%%  Functions for Calculating cost  %%%
        function val = cost(obj,u)
            % Simulate instantaneous cost and dynamics
            z0 = [0; obj.x0];
            opts = odeset('MaxStep',obj.dt);
            [t, z] = ode45(@(t, z)obj.systemAndCostDynamics(t, z, u), [0:obj.dt:obj.T], z0, opts);            
            z = z';
            
            % Calcualte the Terminal Cost
            x_T = z(2:end, end); % Extract terminal state
            phi = obj.terminalCost(x_T);
            
            % Full cost
            val = phi + z(1, end);
        end
        
        function zdot = systemAndCostDynamics(obj, t, z, u)
            % Extract states and initialize derivative
            zdot = zeros(4,1);
            x = z(2:4);
            % u = z(5:6);
            
            % Calculate dynamics of instantaneous cost
            zdot(1) = obj.instantaneousCost(t,x,u);
            
            % Calculate unicycle dynamics
            zdot(2:end) = obj.unicycleDynamics(t,x,u);           
        end
        
        function phi = terminalCost(obj, x)
            e = x(1:2) - obj.qd;
            phi = obj.p5 * (e'*e);
        end
        
        function L = instantaneousCost(obj, t, x, u)
            L_avoid = 0;
            q = x(1:2);
            for i = 1:length(obj.qb)
                s = sqrt((q-obj.qb(:,i))'*(q-obj.qb(:,i)));

                if s <= obj.dmax && s > obj.dmin
                    L_avoid =  obj.p3*(log(obj.dmax-obj.dmin)-log(s - obj.dmin)) + L_avoid;
                elseif s < obj.dmin
                    %L_avoid = obj.p3*200 + L_avoid;
                    L_avoid = L_avoid + inf;
                else
                    L_avoid = L_avoid;
                end
            end
            L = obj.p1*(obj.vd-u(1))^2 + obj.p2*u(2)^2 + L_avoid;
        end
        
        function xdot = unicycleDynamics(obj, t, x, u)
            xdot = obj.dynamicsSimple(t,x, u);
        end
        
        function xdot = dynamicsSimple(obj, t, x, u)
            v = obj.velocityAtTimeInterval(t, u);
            xdot = [v(1)*cos(x(3)); v(1)*sin(x(3)); v(2)];
        end
        
        %%% Function for calculating partial %%%
        function dJ_du = partial(obj, u)
            if obj.numericalPartialLogic
                dJ_du = obj.numericPartial(u);
                return;
            end
            
            
           % Simulate x forward in time
           x_sol = ode45(@(t,x)obj.unicycleDynamics(t,x,u), [0:obj.dt:obj.T], obj.x0);
           
           % Simulate lambda backwards in time
           x_T = deval(x_sol, obj.T);  % final state
           lam_T = obj.terminalPartial(x_T);
           lam_sol = ode45(@(t,lam)obj.lamCostateDynamics(t, lam, x_sol, u), [obj.T:-obj.dt:0], lam_T);
           
           % Extract the initial condition for the lambda costate
           % lam_0 = deval(lam_sol, 0); % Used in the calculation of time
           % intervals
           
           dphi_du = [0 0];
           zeta_tau1 = dphi_du; % dphi_du = [0 0] for all time
           zeta0_sol = ode45(@(t,zeta)obj.zetaCostateDynamics(t, zeta, x_sol, lam_sol, u), [obj.T:-obj.dt:0], zeta_tau1);
           
           
           % Extract the initial condition for the zeta costate
           zeta_time0 = deval(zeta0_sol,0);
           
           % Return the gradient
           dJ_du = zeta_time0';
           
           
        end
        
        function dphi = terminalPartial(obj, x)
            q = x(1:2);
            dphi = obj.p5*[2*(q-obj.qd)' 0];
        end
        
        function lam_dot = lamCostateDynamics(obj, t, lam, x_sol, u)
            x = deval(x_sol, t);
            
            %% Todo: do you want your velocity or your input?
            v = obj.velocityAtTimeInterval(t, u);
            
            % Calculate partials
            [dL_dx, dL_du] = obj.instantaneousPartial(x,v);
            [df_dx, df_du] = obj.dynamicsPartial(x,v);
            
            % Calcualte lambda dot
            lam_dot = -dL_dx' - df_dx'*lam;
        end
        
        function zeta_dot = zetaCostateDynamics(obj, t, zeta_0, x_sol,lam_sol, u)
            x = deval(x_sol, t);
            lam = deval(lam_sol, t);
            
            u = obj.velocityAtTimeInterval(t, u);
            
            % Calculate partials
            [dL_dx, dL_du] = obj.instantaneousPartial(x,u);
            [df_dx, df_du] = obj.dynamicsPartial(x,u);
            
            % Calcualte lambda dot
            zeta_dot = -dL_du' - df_du'*lam;
        end
        
        function [dL_dx, dL_du] = instantaneousPartial(obj, x, u)
            
            dL_dx = 0; % Initialize for each partial calc
            q = x(1:2);
            
            % Sum of the obstacle's avoidance cost
            for i = 1:length(obj.qb)
                % Distance from obstacle
                s = (q-obj.qb(:,i))'*(q-obj.qb(:,i)); 
                
                % Derivative of Weighted log function
                if sqrt(s) <= obj.dmax && sqrt(s) >= obj.dmin
                    dL_ds =  obj.p3*(-1/((s-sqrt(s)*obj.dmin)*2));                    
                elseif sqrt(s) >= obj.dmax
                    dL_ds = 0;
                else
                    dL_ds = -10000000000000;
                end
                
                % Setting up a differential cascade to get dL_dx
                ds_dq = 2*(q-obj.qb(:,i))';
                dq_dx = [eye(2) [0;0]];
                dL_dx = dL_ds*ds_dq*dq_dx + dL_dx;
                
                if sqrt(s) <= obj.dmax && sqrt(s) >= obj.dmin
                    dL_dx2 = obj.getPartialAvoid(x,u,obj.qb(:,i));
                    err = norm(dL_dx' - dL_dx2');
                    if err > .00001
                        %disp('partial error');
                    end
                end
            end
            
            % Weighted partial for maintaining desired velocity and angular
            % velocity
            dL_du = [-2*obj.p1*(obj.vd-u(1)) 2*obj.p2*u(2)];
        end
        
        function dLavoid_dx = getPartialAvoid(obj, x, u, qb)
            q = x(1:2);
            s = norm(q-qb);
            
            % Calculate partials components
            dLavoid_ds = -obj.p3/(s-obj.dmin);
            ds_dq = (q - qb)' / s;
            dq_dx = [1 0 0; 0 1 0];
            
            % Calculate partial
            dLavoid_dx = dLavoid_ds*ds_dq*dq_dx;
        end
        
        function [df_dx, df_du] = dynamicsPartial(obj, x, u)
            df_dx = zeros(3);
            df_dx(1,3) = -u(1)*sin(x(3));
            df_dx(2,3) = u(1)*cos(x(3));
            
            df_du = [cos(x(3)) 0; sin(x(3)) 0; 0 1];
            
        end
        
        function v = velocityAtTimeInterval(obj, t, u)
            v = u;
        end
        
        function step = optimal_step(obj, x)
            step = [0;0];
        end
        
        %%% Plotting functions %%%%
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
                    Z(m,n) = min(obj.zlim(2), costfunction(x));
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
            %contour(X1,X2, Z, 70);
            contour(X1,X2, Z, [.25:.01:.75, .75:1:50]);
            hold on;
        end
        
        function plotState(obj, x)
            if obj.n ~= 3 % Only plot for 2D optimization
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
           [tvec, xvec] = ode45(@(t,x)obj.unicycleDynamics(t,x,u), [0:obj.dt:obj.T], obj.x0);
           xvec = xvec';
           
           % Plot the data 
           if isempty(obj.state_handle)
               figure;
               obj.state_handle = plot(xvec(1,:), xvec(2,:), 'r', 'linewidth', 2); hold on;
               for i = 1:length(obj.qb)
                   plot(obj.qb(1,i), obj.qb(2,i), 'ko', 'linewidth', 2);
                   circle(obj.qb(:,i), obj.dmin, 20, 'r', []);
                   circle(obj.qb(:,i), obj.dmax, 50, 'b', []);
               end
               plot(obj.qd(1), obj.qd(2), 'go', 'linewidth', 4); % Plot the goal position
%               ylim([0 max(xvec(2,:))]);
               obj.state_ax = gca;
               title('Trajectory of unicycle');
           else
               set(obj.state_handle, 'xdata', xvec(1,:), 'ydata', xvec(2,:));
 %              set(obj.state_ax, 'ylim', [0 max(xvec(2,:))]);
           end
        end
        
    end
end

