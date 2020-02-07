classdef MultiAgentScenario < handle
    %MultiAgentScenario implements a scenario with one or more agents
    
    properties
        % Define elements to be simulated
        agents % Cell structure of instances of SingleAgent class 
        n_agents % Number of agents
        world % Instance of the Polygon world class
        
        % Agent indexing
        n_states % Number of states total in all agents
        state_ind % Cell structure containing indices of the states
        x0 % Initial state
        
        % Simulation parameters
        plot_during_sim; % true => plot while simulating (requires euler integration)
        t0 = 0; % Initial time of simulation
        dt = 0.05; % Simulation step size
        tf = 40; % Final time of simulation
        plotters % Cell structure holding instances of the Plotter class
        n_plotters % Number of plotters
        
        % Simulation results
        tmat = [] % Matrix of time values
        xmat = [] % Matrix of state values
        
        % Plotting updates
        T = .2 % Plotting period
        t_latest = tic % Timer for plotting
    end
    
    methods(Abstract)
        plotResults(obj) % Plots the results of the simulation
    end
    
    methods
        function obj = MultiAgentScenario(agents,world,plotters,plot_during_sim)
            % Store input variables
            obj.agents = agents;
            obj.n_agents = length(agents);
            obj.world = world;
            obj.plotters = plotters;
            obj.n_plotters = length(plotters);
            obj.plot_during_sim = plot_during_sim;
            
            % Initialize state indices
            obj.state_ind = cell(obj.n_agents, 1);
            index = 1;
            obj.n_states = 0;
            obj.x0 = [];
            for i = 1:obj.n_agents
                n = length(obj.agents{i}.vehicle.x); % Number of states in agent i
                obj.state_ind{i} = index:(index-1+n); % Indices for agent i
                index = index+n; % Increment the indices
                obj.n_states = obj.n_states + n; % Increment the total number of states
                obj.x0 = [obj.x0; obj.agents{i}.vehicle.x]; % Update x0
            end
        end
        
        function runScenario(obj)
            % Initialize time vector
            obj.tmat = [obj.t0:obj.dt:obj.tf]';
            
            % Initialize plots
            obj.initializePlots();
            pause();
            
            % Simulate forward while plotting
            obj.integrateEuler();
            
            % Plot the results
            obj.plotState(obj.tf);
            obj.plotWorld(obj.tf);
            obj.plotResults();
        end
        
        function processSensors(obj, t, x)
            % Have each agent process its sensors
            for i = 1:obj.n_agents
                obj.agents{i}.processSensors(t, x(obj.state_ind{i}), obj.agents);
            end
        end
    end
    
    %%% Plotting methods
    methods
        function initializePlots(obj)
            figure; 
            % Plot the world
            obj.world.plotWorld(gca);
            
            % Loop through the plotters and initialize the plots
            for k = 1:obj.n_plotters
                obj.plotters{k}.initializePlot(obj.t0);
            end
            
            % Pause for user adjustment
%             pause();
        end
        
        function plotState(obj, t)
            % Loop through the plotters
            for k = 1:obj.n_plotters
                obj.plotters{k}.plot(t);
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
        
        function plotWorld(obj, t)
           %%% By default the function is empty, but it can be inherited to 
           %%% update the world plot
        end
    end
    
    %%% Integration methods
    methods (Access=protected)
        function integrateEuler(obj)
            % Initialize state data
            len = length(obj.tmat);
            obj.xmat = zeros(obj.n_states, len);
            obj.xmat(:,1) = obj.x0;
            
            % Storage variables
            x_t_p_dt = obj.x0; % temporary storage for x(t+dt)
            x_dot = zeros(obj.n_states, 1); % temporary storate for \dot{x}(t)
            
            % Loop through and calculate the state
            for k = 1:len
                % Calculate state update equation
                t = obj.tmat(k);
                x_t = x_t_p_dt; % We have now augmented x(t) to previous x(t);
                
                % Process sensors
                obj.processSensors(t, x_t);
                
                % Calculate the dynamics for each agent
                for i = 1:obj.n_agents
                    u = obj.agents{i}.control(t, x_t(obj.state_ind{i}), obj.agents);
                    x_dot(obj.state_ind{i}) = obj.agents{i}.dynamics(t, x_t(obj.state_ind{i}), u);
                end
                
                % Calculate the euler update
                x_t_p_dt = x_t + obj.dt * x_dot;
                
                % Set the update for each agent
                for i = 1:obj.n_agents
                    obj.agents{i}.vehicle.x = x_t_p_dt(obj.state_ind{i});
                end

                % Store the state
                obj.xmat(:,k) = x_t;
                
                % Plot the state
                if true % obj.isPlotReady()
                    obj.plotState(t);
                    obj.plotWorld(t); 
                    pause(obj.dt/4);
                    %t
                end
            end
        end
        
        function ind = getStateIndex(obj, t)
            ind = (t-obj.t0)/obj.dt;
        end
    end
end

