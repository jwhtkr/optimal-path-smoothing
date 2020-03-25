classdef Unicycle2 < CostClass
    
    properties
        % Cost weights
        p1 = 0.45; % weight on velocity
        p2 = 0.35; % weight on angular velocity
        p3 = 0.35; % weight of avoidance
        p4 = 0.3; % weight of voronoi barrier
        p5 = 1.2; % weight on go to goal
        
%         p1 = 0.85; % weight on velocity
%         p2 = 0.85; % weight on angular velocity
%         p3 = 0.15; % weight of avoidance
%         p4 = 0.15; % weight of voronoi barrier
%         p5 = 1.2; % weight on go to goal
        
        % Operational flags
        numericalPartialLogic = false;
        useEulerIntegration = true;
        useRungeKuttaIntegration = false;
        
        % Cost variables
        qd = []; % Desired position
        xd = []; %Desired State
        qb = []; % Obstacles
        % qb = [[3;3], [4;4]];
        n_obs;
        sig = 10;
        vd = 1; % Desired Velocity
        
        dmin = 0.2; %.25; % Distance from obstacle just before collision
        dmax = 1.15;%1.25;
        dmax2 = .75;
        log_dmax_dmin;
        log_dmax_dmin2;
        
        % Time Variables
        time_collision;     % time of collison
        collision_detection = false;    % collision bool
        T = 3; % Time variable for integration
        tf = 3;% Final time
        dt = 0.1; % Integration stepsize
        t_span % Stores the span for integration
        t_span_rev % Same as t_span but in reverse order
        t_len % Number of time variables
        t_sim = 0.05; % Simulation time
        
        K_vel_ctrl;
        K_point_ctrl;
        agent_num;
        leader_traj;
        trajectory;
        voronoi;
        eps_vel;
        
        % Define the control variables
        ind_v1 = 1;
        ind_w1 = 2;
        ind_time1 = 3;
        ind_v2 = 4;
        ind_w2 = 5;
        ind_time2 = 6;
        ind_v3 = 7;
        ind_w3 = 8;
        ind_time3 = 9;
        %         ind_v4 = 10;
        %         ind_w4 = 11;
        
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
        traj_arc4 = [];
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
            obj.log_dmax_dmin2 = log(obj.dmax2 - obj.dmin);
            % Control Gain
            A = zeros(2);
            B = eye(2);
            Q = eye(2);
%             Q = [10 0; 0 10];
            R = eye(2);
%             R = [1 0; 0 1];
            obj.K_vel_ctrl = lqr(A,B,Q,R);
            obj.eps_vel = .2;
            
            A = [zeros(2) eye(2); zeros(2,4)];
            B = [zeros(2); eye(2)];
            Q = diag([1, 1, 1, 1]);
            R = diag([1, 1]);
            obj.K_point_ctrl = lqr(A, B, Q, R);
            
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
        
        function u = minimize(obj,t_sim,x,u)
            obj.t_sim = t_sim;
            obj.reset(x);
            
            obj.cost(u);
            step = @(u) obj.armijo_step(u);
            u = obj.initialize(u);

            while ~obj.armijo_stop(u) 
                u = u + step(u);
            end

            
            obj.plotTraj(u);
            pause(0.3);
            % Move time horizons one time step closer while command is
            % executed
            if u(obj.ind_time1) >= obj.dt
                u(obj.ind_time1) = u(obj.ind_time1) - obj.dt;
            elseif u(obj.ind_time1) < obj.dt
                u(obj.ind_time1) = 0;
            end
            if u(obj.ind_time2) >= obj.dt
                u(obj.ind_time2) = u(obj.ind_time2) - obj.dt;
            elseif u(obj.ind_time2) < obj.dt
                u(obj.ind_time2) = 0;
            end
            if u(obj.ind_time3) >= obj.dt
                u(obj.ind_time3) = u(obj.ind_time3) - obj.dt;
            elseif u(obj.ind_time3) < obj.dt
                u(obj.ind_time3) = 0;
            end

            
        end
        
        function u0 = initialize(obj,u)
            % Store initial cost
            min_cost = obj.cost(u);
            cost = @(var) obj.cost(var);
            u0 = u;
            
            % Create Velocities to explore
            w1 = -80/obj.T*3*pi/180:100/obj.T*3*pi/180/15:80/obj.T*3*pi/180;
            
            for i = 1:length(w1)
                % Initialize a u to explore, T-tau3 = dt
                u_var = [obj.vd; w1(i);obj.T/3;obj.vd;w1(i);obj.T*2/3;obj.vd;w1(i); obj.T];
%                 obj.plotTraj(u_var);
%                 pause(0.02);
                if min_cost > cost(u_var)
                    u0 = u_var;
                    min_cost = cost(u_var);
                end
            end
            for i = 1:length(w1)
                % Initialize a u to explore, T-tau3 = dt
                u_var = [obj.vd; w1(i);obj.T/3;obj.vd;w1(i);obj.T*2/3;obj.vd;w1(i); obj.T*2/3];
%                 obj.plotTraj(u_var);
%                 pause(0.02);
                if min_cost > cost(u_var)
                    u0 = u_var;
                    min_cost = cost(u_var);
                end
            end
            % Test if the tracking controller is minimal
            if ~(u0(obj.ind_time1) == 0) || ~(u0(obj.ind_time2) == 0) || ~(u0(obj.ind_time3) == 0)
                u_var = [1; 0; 0; 1; 0; 0; 1; 0; 0];
                if min_cost > cost(u_var)
                    u0 = u_var;
                    min_cost = cost(u_var);
                end
            end
            
            if isinf(min_cost)
                obj.collision_detection = true;
                obj.cost(u0);
                
%                 obj.time_collision
                
                if ~obj.collision_detection
                    % Scale parameters to avoid collision
                    s = 1.0%*obj.time_collision/obj.T;
                    u0(obj.ind_v1) = u0(obj.ind_v1)*s;
                    u0(obj.ind_w1) = u0(obj.ind_w1)*s;
                    u0(obj.ind_v2) = u0(obj.ind_v2)*s;
                    u0(obj.ind_w2) = u0(obj.ind_w2)*s;
                    u0(obj.ind_v3) = u0(obj.ind_v3)*s;
                    u0(obj.ind_w3) = u0(obj.ind_w3)*s;
                    u0(obj.ind_time1) = u0(obj.ind_time1)*s;
                    u0(obj.ind_time2) = u0(obj.ind_time2)*s;
                    u0(obj.ind_time3) = u0(obj.ind_time3)*s;
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
            
            % Get cost for velocities
            [v_traj, w_traj] = obj.trajectory.getVelocities(t+obj.t_sim);
            L = obj.p1/2*(v-v_traj)^2 + obj.p2/2*(w-w_traj)^2;
%             L = obj.p1/2*(v-v_traj)^2 + obj.p2/2*(w)^2;
            
            % Get Cost for barrier
%             th_l = obj.leader_traj.getYaw(t+obj.t_sim);
%             q_l = obj.leader_traj.reference_traj(t+obj.t_sim);
%             d = obj.voronoi.dist_to_barrier(q,obj.agent_num,th_l,q_l);
%             for k = 1:length(d)
%                 if d(k) > obj.dmin && d(k) < obj.dmax2
%                     Lvoronoi = obj.p4*(obj.log_dmax_dmin2 - log(d(k) - obj.dmin));
%                     L = L + Lvoronoi;
%                 elseif d(k) < obj.dmin
%                     L = L + inf;
%                     if isnan(L)
%                         L = 0;
%                     end
%                 end
%             end
            
            
            % Get Cost for obstacles
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
            
            if obj.xd(obj.ind_theta) > pi
                obj.xd(obj.ind_theta) = obj.xd(obj.ind_theta) - 2*pi;
            elseif obj.xd(obj.ind_theta) < -pi
                obj.xd(obj.ind_theta) = obj.xd(obj.ind_theta) + 2*pi;
            end
            
            if x(obj.ind_theta) > pi
                x(obj.ind_theta) = x(obj.ind_theta) - 2*pi;
            elseif x(obj.ind_theta) < -pi
                x(obj.ind_theta) = x(obj.ind_theta) + 2*pi;
            end
            
            % Calculate cost as state differenct squared to goal
            h_e = [cos(x(3));sin(x(3))] - [cos(obj.xd(3));sin(obj.xd(3))];
            
            err = x - obj.xd;
            if err(obj.ind_theta) > pi
                err(obj.ind_theta) = err(obj.ind_theta) - 2*pi;
            elseif err(obj.ind_theta) < -pi
                err(obj.ind_theta) = err(obj.ind_theta) + 2*pi;
            end
%             err(3) = norm(h_e);
            phi = obj.p5/2*(err'*err);
        end
        
        function zdot = costAndStateDynamics(obj, t, z, u)
            % Extract state and cost
            x = z(1:obj.n);
            
            % Calculate Dual Mode time derivative
            if t > u(obj.ind_time3)
                xdot = obj.unicycleTracking(t,x,u);
            else
                xdot = obj.unicycleDynamics(t, x, u);
            end
            
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
            
            % Simulate state forward in time from 0 to tau3
            obj.setTimeSpan(u,0);
            x_sol = obj.integrate(@(t,x)obj.unicycleDualModeDynamics(t, x, u), obj.x0, true, false);
%             obj.setTimeSpan(u,5);
%             x_sol1 = obj.integrate(@(t,x)obj.unicycleDynamics(t, x, u), obj.x0, true, false);
%             x_tau3 = x_sol1(:,end); %obj.evalIntResult(x_sol1,u(obj.ind_time3));
%             
%             % Simulate state forward in time from tau3 to T
%             obj.setTimeSpan(u,4);
%             x_sol2 = obj.integrate(@(t,x)obj.unicycleTracking(t, x, u), x_tau3, true, false);
%             
%             x_sol = [x_sol1 x_sol2];
%             if length(x_sol) > obj.T/obj.dt+1
%                 x_sol = [x_sol1 x_sol2(:,2:end)];
%             end
            
            % Create costateT initial conditions
            xT = obj.evalIntResult(x_sol, obj.T);
            lamT = obj.terminalStatePartial(xT)';
            xiT = zeros(2,1);
            costateT = [lamT; xiT];
            
            % Simulate costates backward in time Tf to tau3
            obj.setTimeSpan(u,4);
            costates_T = obj.integrate(@(t,costate)obj.costateDynamics(t, costate, x_sol, u), costateT, false, false);   
            
            % Evaluate Results, extract xi values and reset for next time
            % interval
            costate3 = costates_T(:,1); %obj.evalIntResult(costates_T, u(obj.ind_time3));
            xi3 = costate3(obj.n+1:end);
         
            lam3 = costate3(1:obj.n);
            % Setting xi inital condition to 0
            costate3(4:end) = 0;
            
            % Simulate costates backwards in time tau3 to tau2
            obj.setTimeSpan(u,3);
            costates_3 = obj.integrate(@(t,costate)obj.costateDynamics(t, costate, x_sol, u), costate3, false, false);
            
            % Evaluate Results, extract xi values and reset for next time
            % interval
            if ~isempty(costates_3)
                costate2 = costates_3(:,1); %obj.evalIntResult(costates_3, u(obj.ind_time2));
                xi2 = costate2(obj.n+1:end);
                lam2 = costate2(1:obj.n);
                % Setting xi inital condition to 0
                costate2(4:end) = 0;
            else
                % If previous time window doesn't exist set initials to previous
                % values
                costate2 = costate3; 
                xi2 = xi3;
                lam2 = lam3;
            end
            
            % Simulate costates backwards in time tau2 to tau1
            obj.setTimeSpan(u,2);
            costates_2 = obj.integrate(@(t,costate)obj.costateDynamics(t, costate, x_sol, u), costate2, false, false);

            % Evaluate Results, extract xi values and reset for next time
            % interval
            if ~isempty(costates_2)
                costate1 = costates_2(:,1); %obj.evalIntResult(costates_2, u(obj.ind_time1));
                xi1 = costate1(obj.n+1:end);
                lam1 = costate1(1:obj.n);
                % Setting xi inital condition to 0
                costate1(4:end) = 0;
            else
                % If previous time window doesn't exist set initials to previous
                % values
                costate1 = costate2; 
                xi1 = xi2;
                lam1 = lam2;
            end
            
            % Simulate costates backwards in time tau1 to 0
            obj.setTimeSpan(u,1);
%             z_sol = x_sol(:,floor(obj.t_span/obj.dt)+1);
            costates_1 = obj.integrate(@(t,costate)obj.costateDynamics(t, costate, x_sol, u), costate1, false, false);
%             z_sol = [];
            
            % Evaluate results, extract xi values and reset time span to 0
            % to T
            if ~isempty(costates_1)
                costate0 = costates_1(:,1); %obj.evalIntResult(costates_1, 0);
                xi0 = costate0(obj.n+1:end);
                % lam0 = costate0(1:obj.n);
            else
                % If previous time window doesn't exist set initials to previous
                % values
                costate0 = costate1; 
                xi0 = xi1;
                % lam0 = lam1;
            end
                
            % Reset t_span from 0 to T
            obj.setTimeSpan(u,0);
            
            
            % Calculate cost forward in time
            z0 = [obj.x0; 0];
            cost = obj.integrate(@(t,z)obj.costAndStateDynamics(t,z,u), z0, true, true);
            % Extract Cost at key times
            cost0 = obj.evalIntResult(cost,0);
            cost1 = obj.evalIntResult(cost,u(obj.ind_time1));
            cost2 = obj.evalIntResult(cost,u(obj.ind_time2));
            cost3 = obj.evalIntResult(cost,u(obj.ind_time3));
            cost4 = obj.evalIntResult(cost,obj.T);
            
            % Calculate partial of J with respect to tau
            dJ_dtau1 = cost0(obj.ind_cost) - cost1(obj.ind_cost) + lam1'*(cost0(1:obj.n) - cost1(1:obj.n));
            dJ_dtau2 = cost1(obj.ind_cost) - cost2(obj.ind_cost) + lam2'*(cost1(1:obj.n) - cost2(1:obj.n));
            dJ_dtau3 = cost2(obj.ind_cost) - cost3(obj.ind_cost) + lam3'*(cost2(1:obj.n) - cost3(1:obj.n));
            
            if isnan(xi0(1)) || isnan(xi0(2)) || isnan(dJ_dtau1) || isnan(xi1(1)) || isnan(xi1(2)) || isnan(dJ_dtau2) || isnan(xi2(1)) || isnan(xi2(2)) || isnan(dJ_dtau3)
                test = 1;
            end
            % Compile Partial 1x9
            dJ_du = [xi0' dJ_dtau1 xi1' dJ_dtau2 xi2' dJ_dtau3];
        end
        
        function dphi_dx = terminalStatePartial(obj, x)
            % Extract states and velocities
            
            
            if obj.xd(obj.ind_theta) > pi
                obj.xd(obj.ind_theta) = obj.xd(obj.ind_theta) - 2*pi;
            elseif obj.xd(obj.ind_theta) < -pi
                obj.xd(obj.ind_theta) = obj.xd(obj.ind_theta) + 2*pi;
            end
            
            if x(obj.ind_theta) > pi
                x(obj.ind_theta) = x(obj.ind_theta) - 2*pi;
            elseif x(obj.ind_theta) < -pi
                x(obj.ind_theta) = x(obj.ind_theta) + 2*pi;
            end
            
            err = x - obj.xd;
            
            if err(obj.ind_theta) > pi
                err(obj.ind_theta) = err(obj.ind_theta) - 2*pi;
            elseif err(obj.ind_theta) < -pi
                err(obj.ind_theta) = err(obj.ind_theta) + 2*pi;
            end
            
            % Calculate partial
            dphi_dx = obj.p5.*err';
        end
        
        function dL_dx = instStatePartial(obj, t, x, u)
            % Extract states
            [v, w] = obj.getVelocities(x);
            q = obj.getPosition(x);
            
            
            dL_dx = zeros(1, obj.n);
            dq_dx = [eye(2), [0; 0], [0; 0], [0; 0]];
            
%             % Calculate Distance from barrier
%             th_l = obj.leader_traj.getYaw(t+obj.t_sim);
%             q_l = obj.leader_traj.reference_traj(t+obj.t_sim);
%             [d_barrier, n] = obj.voronoi.dist_and_norm(q,obj.agent_num,th_l,q_l);
%             
%             dLvor_dd = 0;
%             for k = 1:length(d_barrier)
%                 % Calculate Log Barrier
%                 if d_barrier(k) > obj.dmin && d_barrier(k) < obj.dmax2
%                     dLvor_dd = -obj.p4/(d_barrier(k)-obj.dmin);
%                 elseif d_barrier(k) < obj.dmin
%                     dLvor_dd = -inf;
%                 end
%                 
%                 % Calculate distance partial based on position
%                 dd_dq = n(:,k)';
%                 
%                 % Calculate contribution of avoidance
%                 dLvor_dx = dLvor_dd * dd_dq * dq_dx;
%                 dL_dx = dL_dx + dLvor_dx;
%             end
            
            % Calculate necesssary derivatives
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
            [v_traj, w_traj] = obj.trajectory.getVelocities(t+obj.t_sim);
            dL_dx = dL_dx + [0 0 0 obj.p1*(v-v_traj) obj.p2*(w-w_traj)];
%             dL_dx = dL_dx + [0 0 0 obj.p1*(v-v_traj) obj.p2*(w)];
            
        end
        
        function dL_du = instInputPartial(obj, t, x, u)
            % Partial of L to control
            dL_du = [0 0];
        end
        
        function df_dx = dynamicsStatePartial(obj, t, x, u)
            % Extract states and velocities
            q = obj.getPosition(x);
            [v,w] = obj.getVelocities(x);
            theta = x(obj.ind_theta);
            
            % Calculate partial
            if t > u(obj.ind_time3)
                c = cos(theta);
                s = sin(theta);
                e = obj.eps_vel;
                k1 = obj.K_point_ctrl(1,1);
                k2 = obj.K_point_ctrl(1,3);
                [q_des, qd_des, qdd_des] = obj.trajectory.reference_traj(t);
                
                % Partial of unicycle epsilon point following with respect to state
                % x = [q; theta; v; w];
                z1 = (qdd_des(1) - k1*(q(1) - q_des(1) + e*c) + k2*(qd_des(1) - v*c + e*w*s));
                z2 = (-qdd_des(2) +k1*(q(2) - q_des(2) + e*s) + k2*(-qd_des(2) +v*s + e*w*c))/e;
                dz1 = k1*e*s + k2*v*s + k2*e*w*c;
                dz2 = (k1*e*c + k2*v*c - k2*e*w*s)/e;
                
                du1_dx = [-k1*c, k1*s/e, -s*z1 + c*z2 + c*dz1 + s*dz2, k2*(s^2/e - c^2), k2*(c*s + e*c*s);
                    -k1*s,-k1*c/e,  c*z1 + s*z2 + s*dz1 - c*dz2,-k2*(c*s + c*s/e), k2*(e*s^2 - c^2)];
                du2_dx = [0 0 0 0 -2*w; 0 0 0 w v];
                du_dx = du1_dx - du2_dx;
                df_dx = [0 0 -v*sin(theta) cos(theta) 0; 0 0 v*cos(theta) sin(theta) 0; 0 0 0 0 1; du_dx];
            else
                df_dx = [0 0 -v*sin(theta) cos(theta) 0; 0 0 v*cos(theta) sin(theta) 0; 0 0 0 0 1; 0 0 0 -obj.K_vel_ctrl(1,1) 0; 0 0 0 0 -obj.K_vel_ctrl(2,2)];
                % df_dx = [0 0 -v*sin(theta) cos(theta) 0; 0 0 v*cos(theta) sin(theta) 0; 0 0 0 0 1; 0 0 0 0 0; 0 0 0 0 0];
            end
        end
        
        function df_du = dynamicsInputPartial(obj, t, x, u)
            % Calculate partial
            if t > u(obj.ind_time3)
                % Partial for unicycle epsilon tracking of control [u_v;
                % u_w]
                df_du = [0, 0; 0, 0; 0, 0; 1 0; 0 1];
            else
                % Partial for arcbased unicycleof control [u_v; u_w]
                df_du = [0, 0; 0, 0; 0, 0; obj.K_vel_ctrl];
            end
        end
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%% State and Dynamics functions %%%%
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function [v, w] = getDesiredVelocity(obj,u,t)
            if t < u(obj.ind_time1)
                v = u(obj.ind_v1);
                w = u(obj.ind_w1);
            elseif t >= u(obj.ind_time1) && t < u(obj.ind_time2)
                v = u(obj.ind_v2);
                w = u(obj.ind_w2);
            elseif t >= u(obj.ind_time2) && t <= u(obj.ind_time3)
                v = u(obj.ind_v3);
                w = u(obj.ind_w3);
            else
                v = 1;
                w = 0;
            end
            
        end
        
        function [v, w] = getVelocities(obj, x)
            v = x(obj.ind_v);
            w = x(obj.ind_w);
        end
        
        function q = getPosition(obj, x)
            q = [x(obj.ind_x); x(obj.ind_y)];
        end
        
        function p = p_global(obj,x)
            q = obj.getPosition(x);
            d = norm(q - obj.qd);
            p = 1 - exp(-d^2/obj.sig^2);
        end
        
        function setGoal(obj, q)
            obj.qd = q;
        end
        
        function xdot = unicycleDualModeDynamics(obj,t,x,u)
            if t > u(obj.ind_time3) || u(obj.ind_time3) == 0
                xdot = obj.unicycleTracking(t,x,u);
            else
                xdot = obj.unicycleDynamics(t,x,u);
            end
        end
        
        function xdot = unicycleDynamics(obj, t, x, u)
            % Extract states and velocities
            [v, w] = obj.getVelocities(x);
            [vd, wd] = obj.getDesiredVelocity(u,t);
            theta = x(obj.ind_theta);
            
            % Calculate derivative
            xdot = zeros(obj.n, 1);
            xdot(obj.ind_x) = v*cos(theta);
            xdot(obj.ind_y) = v*sin(theta);
            xdot(obj.ind_theta) = w;
            xdot(obj.ind_v) = -obj.K_vel_ctrl(1,1)*(v-vd);
            xdot(obj.ind_w) = -obj.K_vel_ctrl(2,2)*(w-wd);
        end
        
        function xdot = unicycleTracking(obj, t, x, u)
            % Extract states and velocities
            [v, w] = obj.getVelocities(x);
            theta = x(obj.ind_theta);
            
            % Calculate derivative
            xdot = zeros(obj.n, 1);
            xdot(obj.ind_x) = v*cos(theta);
            xdot(obj.ind_y) = v*sin(theta);
            xdot(obj.ind_theta) = w;
            
            u_control = obj.trackControl(t+obj.t_sim-obj.dt,x);
            xdot(obj.ind_v) = u_control(1);
            xdot(obj.ind_w) = u_control(2);
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
            obj.setTimeSpan(u,5);
            xvec1 = obj.integrate(@(t,x)obj.unicycleDynamics(t, x, u), obj.x0, true, false);
            
            x_tau3 = xvec1(:,end); %obj.evalIntResult(xvec1,u(obj.ind_time3));
            obj.setTimeSpan(u,4);
            xvec2 = obj.integrate(@(t,x)obj.unicycleTracking(t, x, u), x_tau3, true, false);
            
            
            xvec = [xvec1 xvec2];
            obj.setTimeSpan(u,0);
            tvec = obj.t_span;
            j = 1;
            k = 1;
            iter = 1;
            xarc1 = [];
            xarc2 = [];
            xarc3 = [];
            xarc4 = [];
            for i = 1:length(tvec)
                if tvec(i) < u(obj.ind_time1)
                    xarc1(:,i) = xvec(:,i);
                elseif tvec(i) >= u(obj.ind_time1) && tvec(i) <= u(obj.ind_time2)
                    xarc2(:,j) = xvec(:,i);
                    j = j + 1;
                elseif tvec(i) > u(obj.ind_time2) && tvec(i) <= u(obj.ind_time3)
                    xarc3(:,k) = xvec(:,i);
                    k = k + 1;
                elseif tvec(i) > u(obj.ind_time3) && tvec(i) <= obj.T
                    xarc4(:,iter) = xvec(:,i);
                    iter = iter + 1;
                end
            end
            
            % Plot the data
            if isempty(obj.traj_arc1) && ~isempty(xarc1)
                obj.traj_arc1 = plot(xarc1(1,:), xarc1(2,:), 'g', 'linewidth', 2);
            elseif ~isempty(xarc1)
                set(obj.traj_arc1, 'xdata', xarc1(1,:), 'ydata', xarc1(2,:));
            elseif isempty(xarc1) && ~isempty(obj.traj_arc1)
                set(obj.traj_arc1, 'xdata', 0, 'ydata', 0);
            end
            if isempty(obj.traj_arc2) && ~isempty(xarc2)
                obj.traj_arc2 = plot(xarc2(1,:), xarc2(2,:), 'r', 'linewidth', 2);
            elseif ~isempty(xarc2)
                set(obj.traj_arc2, 'xdata', xarc2(1,:), 'ydata', xarc2(2,:));
            elseif isempty(xarc2) && ~isempty(obj.traj_arc2)
                set(obj.traj_arc2, 'xdata', 0, 'ydata', 0);
            end
            if isempty(obj.traj_arc3) && ~isempty(xarc3)
                obj.traj_arc3 = plot(xarc3(1,:), xarc3(2,:), 'b', 'linewidth', 2);
            elseif ~isempty(xarc3)
                set(obj.traj_arc3, 'xdata', xarc3(1,:), 'ydata', xarc3(2,:));
            elseif isempty(xarc3) && ~isempty(obj.traj_arc3)
                set(obj.traj_arc3, 'xdata', 0, 'ydata', 0);
            end
            if isempty(obj.traj_arc4) && ~isempty(xarc4)
                obj.traj_arc4 = plot(xarc4(1,:), xarc4(2,:), 'k', 'linewidth', 2);
            elseif ~isempty(xarc4)
                set(obj.traj_arc4, 'xdata', xarc4(1,:), 'ydata', xarc4(2,:));
            elseif isempty(xarc4) && ~isempty(obj.traj_arc4)
                set(obj.traj_arc4, 'xdata', 0, 'ydata', 0);
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
            elseif obj.useRungeKuttaIntegration
                z_sol = obj.rungeKuttaIntegration(dynamics, z0, forward);
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
        
        function zvec = rungeKuttaIntegration(obj, dynamics, z0, forward)
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
            del_t2 = del_t/2;
            del_t6 = del_t/6;
            
            % Perform integration
            for k = 2:obj.t_len
%                 z_act = z_act + dynamics(t(k), z_act)*del_t;
                z_t = z_act + del_t2*dynamics(t(k),z_act);
                
                dz_t = dynamics(t(k),z_t);
                z_t = z_act + del_t2*dz_t;
                
                dz_m = dynamics(t(k),z_t);
                z_t = z_act + del_t*dz_m;
                dz_m = dz_m + dz_t;
                dz_t = dynamics(t(k),z_t);
                
                z_act = z_act + del_t6*(dynamics(t(k),z_act)+dz_t+2*dz_m);
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
            if obj.useEulerIntegration || obj.useRungeKuttaIntegration
%                 ind = find(obj.t_span == t);
                ind = round(t/obj.dt)+1;
                if isempty(ind)
                    ind = floor(t/obj.dt)+1;
                end
                try
                    z = z_sol(:, ind);
                catch
                    disp('Error!');
                end
                
            else
                z = deval(z_sol, t);
            end
        end
        
        function setTimeSpan(obj, u, logic)
            switch logic
                case 1
                    obj.t_span = 0:obj.dt:u(obj.ind_time1);
                    obj.t_span_rev = flip(obj.t_span);
                    obj.t_len = length(obj.t_span);
                case 2
                    obj.t_span = u(obj.ind_time1):obj.dt:u(obj.ind_time2);
                    obj.t_span_rev = flip(obj.t_span);
                    obj.t_len = length(obj.t_span);
                case 3
                    obj.t_span = u(obj.ind_time2):obj.dt:u(obj.ind_time3);
                    obj.t_span_rev = flip(obj.t_span);
                    obj.t_len = length(obj.t_span);
                case 4
                    obj.t_span = u(obj.ind_time3):obj.dt:obj.T;
                    obj.t_span_rev = flip(obj.t_span);
                    obj.t_len = length(obj.t_span);
                case 5
                    obj.t_span = 0:obj.dt:u(obj.ind_time3);
                    obj.t_span_rev = flip(obj.t_span);
                    obj.t_len = length(obj.t_span);
                case 0
                    obj.t_span = 0:obj.dt:obj.T;
                    obj.t_span_rev = flip(obj.t_span);
                    obj.t_len = length(obj.t_span);
            end
        end
        
        function [xvec, xT] = getTerminalPosition(obj, x0, u)
            
            %z_sol = ode45(@(t,z)obj.costAndStateDynamics(t,z,u), [0:obj.dt:obj.T], z0, opts);
            xvec = obj.integrate(@(t,x)obj.unicycleDynamics(t, x, u), x0, true, false);
            
            % Extract the final value for the cost
            xT = obj.evalIntResult(xvec, obj.T);
        end
        
        function [q_er, qdot_er, qddot_er] = epislonTrajectory(obj, t)
            % Calculate Reference Trajectory
            [q_r, qdot_r, qddot_r, qdddot_r, qddddot_r] = obj.trajectory.reference_traj(t);
            
            % Calculate Reference Trajectory Parameters
            psi_r = atan2(qdot_r(2),qdot_r(1));
            v_r = norm(qdot_r);
            w_r = (qdot_r(1)*qddot_r(2) - qdot_r(2)*qddot_r(1))/v_r^2;
            v = [v_r; w_r];
            a_r = (qdot_r(1)*qddot_r(1) + qdot_r(2)*qddot_r(2))/v_r;
            alpha_r = (qdot_r(1)*qdddot_r(2) - qdot_r(2)*qdddot_r(1))/v_r^2 - 2*a_r*w_r/v_r;
            a = [a_r; alpha_r];
            eps = obj.eps_vel;
            
            
            % Create Algebraic relations to epsilon trajectory
            R_er = [cos(psi_r) -eps*sin(psi_r); sin(psi_r) eps*cos(psi_r)];
            w_hat_r = [0 -eps*w_r; w_r/eps 0];
            
            % Create Epsilon Trajectory
            q_er = q_r + eps*[cos(psi_r);sin(psi_r)];
            qdot_er = R_er*v;
            qddot_er = R_er*w_hat_r*v + R_er*a;
            
        end
        
        %%%%%%%%%%%%%%%%%%%%%% Control Functions %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function u = trackControl(obj, t, x)
            eps = obj.eps_vel;
            % Get states
            x_pos = x(obj.ind_x);
            y_pos = x(obj.ind_y);
            [v, w] = obj.getVelocities(x);
            th = x(obj.ind_theta);
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
            
            % Get epsilon trajectory
            [qd, qd_dot, qd_ddot] = obj.epislonTrajectory(t);
            
            % Calculate point control
            u_point = -obj.K_point_ctrl*(q - [qd; qd_dot]) + qd_ddot;
            
            % Calculate the control inputs
            u = R_e_inv*u_point - w_hat_e*[v; w];
        end
        
    end
end

