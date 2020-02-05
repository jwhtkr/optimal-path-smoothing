classdef MultiReferenceAvoidScenario < MultiAgentScenario
    %MultiReferenceAvoidScenario Implements several agents which track a
    %reference as individual agents
    
    properties
        qd_mat % Stores the desired trajectory over time, used for plotting results,
               % The matrix has 2 x n_agents number of rows (2 for each
               % agent)
    end
    
    methods
        function obj = MultiReferenceAvoidScenario(veh, world, waypoints, x0)
            %MultiReferenceAvoidScenario Construct an instance of this class
            %   veh : instance to a vehicle class to be used to create the
            %   agents
            %   world: instance of the polygon world class
            %   waypoints: Cell structure of waypoints, one for each agent
            %   to be run
            %   x0: Cell structure of initial states, one for each agent to
            %   be run
            
            
            % Define desired trajectory pamameters
            dt = 0.01;
            vd = 1;
            k_max = 5; % Maximum curvature
            sig_max = 5; % Maximum change in curvature
            
            % Create each agent and its corresponding plotter
            n_agents = length(x0);
            agents = cell(n_agents, 1);
            plotters = {};
            for i = 1:length(agents)
                % Create the agents
                veh_i = veh(x0{i});
                traj = CCPathGenerator(waypoints{i}, vd, dt, k_max, sig_max).traj;
                agents{i} = ReferenceAvoidAgent(veh_i, world, traj);
                
                % Create a vehicle plotter
                plotters{end+1} = SingleAgentPlotter(veh_i);
                
                % Createa a plotter for the desired position
                plotters{end+1} = PositionPlotter(@(t)agents{i}.ReferenceTraj(t));
            end
            
            % Initialize the object
            obj = obj@MultiAgentScenario(agents, world, plotters, true);
            
        end
        
        function initializePlots(obj)
            % Plot normal plots
            initializePlots@MultiAgentScenario(obj);
            
            % Plot the reference trajectory for each agent
            tmat = [obj.t0:obj.dt:obj.tf]';
            len = length(tmat);
            obj.qd_mat = zeros(2*obj.n_agents, len);
            ind_q_i = 1:2; % Iteratively stores the position indices for each agent
            for i = 1:obj.n_agents
                % Build the trajectory
                for k = 1:len
                    [obj.qd_mat(ind_q_i,k), ~, ~] = obj.agents{i}.ReferenceTraj(tmat(k));
                end
                
                % Plot the trajectory
                plot(obj.qd_mat(ind_q_i(1),:), obj.qd_mat(ind_q_i(2),:), ':', 'linewidth', 2);
                
                % Update the position indices
                ind_q_i = ind_q_i + 2; % Add two to adjust for the two positions
            end
            
        end
        
        function plotResults(obj)
            ind_q_d = 1:2; % Stores the indices of agent i within q_d
            for i = 1:obj.n_agents
                % Create a figure for the actual and desired positions
                figure;
                
                % Get state indices
                x_ind = obj.state_ind{i}(1);
                y_ind = obj.state_ind{i}(2);
                
                % Plot the x position vs desired position
                subplot(2, 1, 1);
                plot(obj.tmat, obj.qd_mat(ind_q_d(1), :), ':r', 'linewidth', 3); hold on;
                plot(obj.tmat, obj.xmat(x_ind, :), 'b', 'linewidth', 2);
                ylabel('x position');
                
                % Plot the y position vs desired position
                subplot(2, 1, 2);
                plot(obj.tmat, obj.qd_mat(ind_q_d(2), :), ':r', 'linewidth', 3); hold on;
                plot(obj.tmat, obj.xmat(y_ind, :), 'b', 'linewidth', 2);
                ylabel('y position');
                xlabel('time (s)');
                
                % Upate the position indices for the next agent
                ind_q_d = ind_q_d + 2;
            end
        end
    end
end



