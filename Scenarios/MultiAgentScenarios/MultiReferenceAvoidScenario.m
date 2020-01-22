classdef MultiReferenceAvoidScenario < MultiAgentScenario
    %MultiReferenceAvoidScenario Implements several agents which track a
    %reference as individual agents
    
    properties
        
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
            
            dt = 0.01;
            
            % Create each agent and its corresponding plotter
            n_agents = length(x0);
            agents = cell(n_agents, 1);
            plotters = {};
            for i = 1:length(agents)
                % Create the agents
                veh_i = veh(x0{i});
                agents{i} = ReferenceAvoidAgent(veh_i, world, waypoints{i}, dt);
                
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
            for i = 1:obj.n_agents
                % Build the trajectory
                qd = zeros(2, len);                
                for k = 1:len
                    [qd(:,k), ~, ~] = obj.agents{i}.ReferenceTraj(tmat(k));
                end
                
                % Plot the trajectory
                plot(qd(1,:), qd(2,:), ':', 'linewidth', 2);
            end
            
        end
        
        function plotResults(obj)
            warning("plotResults() not yet implemented");
        end
    end
end



