classdef MultiScenario < handle
    
    
    properties
        % Define elements to be simulated
        vl;
        agents;
        n_agents;
        vehicle % Instance of teh Vehicle class
        world % Instance of the Polygon world class
        worldEmpty
        
        % Simulation parameters
        plot_during_sim; % true => plot while simulating (requires euler integration)
        t0 = 0; % Initial time of simulation
        dt = 0.05; % Simulation step size
        tf = 15; % Final time of simulation
        
        % Simulation results
        tmat = [] % Matrix of time values
        xmat = [] % Matrix of state values
        
        % Plotting updates
        T = .2 % Plotting period
        t_latest = tic % Timer for plotting
        video
        
        % Index variables
        x_ind % x position index
        y_ind % y position index
        q_ind % 2D position index
    end
    
    properties (SetAccess = protected, GetAccess = public)
        h_state_traj = [];
        traj = [];
    end
    
    methods (Abstract)
        u = control(obj, t, x); % Calculates the control to the vehicle        
    end
    
    methods
        function obj = MultiScenario(leader,agents,world,plot_during_sim)
            obj.world = world;
            obj.worldEmpty = EmptyWorld();
            obj.plot_during_sim = plot_during_sim;
            obj.vl = leader;
            obj.agents = agents;
            obj.n_agents = length(agents);
            % Get the indices for the position states for easy access
            obj.x_ind = leader.vehicle.kinematics.x_ind;
            obj.y_ind = leader.vehicle.kinematics.y_ind;
            obj.q_ind = [obj.x_ind; obj.y_ind];
            
            % Initialize the obstacle detections
%             obj.vl.vehicle.getObstacleDetections(obj.world);
        end
        
        function runScenario(obj)
            
            % Initialize plots
            obj.initializeStatePlot();
            
            % Simulate forward while plotting
            obj.integrateEuler();
            
            % Plot the results
            obj.plotState(obj.tf);
            obj.plotWorld(obj.tf);
            obj.publishVideo();
            obj.plotResults();
        end
        
        
        %%%%%%  Plotting functions %%%%%%%%%%%
        function plotState(obj, t)
            if isempty(obj.h_state_traj)
               warning('Attempting to plot uninitialized plots');
               return;
            end
            
            % Plot the state trajectory
            ind = floor(obj.getStateIndex(t));
            set(obj.h_state_traj, 'xdata', obj.xmat(obj.x_ind, 1:ind), 'ydata', obj.xmat(obj.y_ind,1:ind));
            
            % Plot the vehicle
            obj.vl.plotFormation();
            
            for k = 1:obj.n_agents
                obj.agents(k).vehicle.plotVehicle();
            end
            
            obj.video = [obj.video getframe(gcf)];
            
            
        end
        
        function publishVideo(obj)
            vid = VideoWriter('FormationPMPC','Motion JPEG AVI');
            open(vid);
            writeVideo(vid,obj.video)
            close(vid)
        end

        
        function initializeStatePlot(obj)
            figure; 
            % Plot the world
            obj.world.plotWorld(gca);
            
            % Initialize the state trajectory
            obj.h_state_traj = plot(0, 0, ':b', 'linewidth', 2); hold on;
            
            % Plot the vehicle
            obj.vl.initializePlots(gca);
            
            for k = 1:obj.n_agents
                obj.agents(k).vehicle.kinematics.plot_path = 0;
                obj.agents(k).vehicle.initializePlots(gca);
            end
            
            pause();
        end
        
        function plotResults2(obj)
            % Create the figure and get additional data
            figure;
            
            % Plot x trajectory
            subplot(3,1,1);
            plot(obj.tmat, obj.xmat(obj.x_ind,:), 'b', 'linewidth', 2);
            ylabel('x_1(t)');
            
            % Plot y trajectory 
            subplot(3,1,2);
            plot(obj.tmat, obj.xmat(obj.y_ind,:), 'b', 'linewidth', 2);
            ylabel('x_2(t)');

            % Calculate the control vs time
            ctrl = zeros(2,length(obj.tmat));
            for k = 1:length(obj.tmat)
                ctrl(:,k) = obj.control(obj.tmat(k), obj.xmat(:,k));
            end
            
            % Plot the control vs time
            subplot(3,1,3);
            plot(obj.tmat, ctrl(1,:), 'g', 'linewidth', 3); hold on;
            plot(obj.tmat, ctrl(2,:), 'b', 'linewidth', 3); 
            ylabel('Inputs');
            legend('u_v', 'u_\omega');            
            xlabel('Time (s)');
        end
        
        function plotResults(obj)
            % Create the figure and get additional data
            figure;
            
            
            % Calculate the control and velocities vs time
            t_len = length(obj.tmat);
            ctrl = zeros(2,t_len);
            v_vec = zeros(1, t_len);
            w_vec = zeros(1, t_len);
            for k = 1:t_len
                ctrl(:,k) = obj.control(obj.tmat(k), obj.xmat(:,k));
                [v_vec(k), w_vec(k)] = obj.vehicle.kinematics.getVelocities(obj.tmat(k), obj.xmat(:,k), ctrl(:,k));
            end
            
            % Plot v and desired
            subplot(3,1,1);
            %plot([obj.tmat(1) obj.tmat(end)],[obj.v_d obj.v_d], ':r', 'linewidth', 2); hold on;
            plot(obj.tmat, v_vec, 'b', 'linewidth', 2);
            ylabel('v(t)');
            
            % Plot w and desired
            subplot(3,1,2);
            %plot([obj.tmat(1) obj.tmat(end)],[obj.w_d obj.w_d], ':r', 'linewidth', 2); hold on;
            plot(obj.tmat, w_vec, 'b', 'linewidth', 2);
            ylabel('\omega(t)');

            % Plot the control vs time
            subplot(3,1,3);
            plot(obj.tmat, ctrl(1,:), 'g', 'linewidth', 3); hold on;
            plot(obj.tmat, ctrl(2,:), 'b', 'linewidth', 3); 
            ylabel('Inputs');
            legend('u_v', 'u_\omega');            
            xlabel('Time (s)');
        end
        
    end
    
    methods (Access=protected)
        function integrateODE(obj)
            % Integrates the vehicle from the initial to final times
            
            % Integrate forward in time
            [t_result, x_result] = ode45(@(t,x)obj.vehicle.kinematics.kinematics(t, x, obj.control(t,x)), [obj.t0:obj.dt:obj.tf], obj.vehicle.x);

            % Transpose the outputs
            obj.tmat = t_result';
            obj.xmat = x_result'; 
            
            % Store the final state as the state of the vehicle
            obj.vehicle.x = obj.xmat(:,end);
        end
        
        function integrateEuler(obj)
            % Initialize state data
            obj.tmat = [obj.t0:obj.dt:obj.tf]';
            len = length(obj.tmat);
            obj.xmat = zeros(length(obj.vl.vehicle.x), len, obj.n_agents+1);
            obj.xmat(:,1,1) = obj.vl.vehicle.x;
            for k = 1:obj.n_agents
                obj.xmat(:,1,k+1) = obj.agents(k).vehicle.x;
            end

            % Loop through and calculate the state
            for k = 1:len
                % Calculate state update equation
                t = obj.tmat(k);
                u = obj.control(t,obj.vl.vehicle.x);
                xdot = obj.vl.vehicle.kinematics.kinematics(t, obj.vl.vehicle.x, u(:,1));

                % Update the state
                obj.vl.vehicle.x = obj.vl.vehicle.x + obj.dt * xdot;

                % Store the state
                obj.xmat(:,k,1) = obj.vl.vehicle.x;
                
                obj.vl.vehicle.getObstacleDetections(obj.worldEmpty);
                for i = 1:obj.n_agents
                    xdot = obj.vl.vehicle.kinematics.kinematics(t, obj.agents(i).vehicle.x, u(:,i+1));

                    % Update the state
                    obj.agents(i).vehicle.x = obj.agents(i).vehicle.x + obj.dt * xdot;

                    % Store the state
                    obj.xmat(:,k,i+1) = obj.agents(i).vehicle.x;

                    % Get the sensor measurements
                    obj.agents(i).vehicle.getObstacleDetections(obj.world);
                end
                
                % Plot the state
                if obj.isPlotReady()
                    obj.plotState(t);
                    obj.plotWorld(t);
                    pause(obj.dt/4); % dt/4 to allow for computation time as well
                end
            end
        end
        
        function result = isPlotReady(obj)
            if obj.plot_during_sim && toc(obj.t_latest) > obj.T
                obj.t_latest = tic;
                result = true;
            else
                result = false;
            end
        end
        
        function ind = getStateIndex(obj, t)
            ind = (t-obj.t0)/obj.dt;
        end
        
        function plotWorld(obj, t)
           %%% By default the function is empty, but it can be inherited to 
           %%% update the world plot
        end
    end
end

