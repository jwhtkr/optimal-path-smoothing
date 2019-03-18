classdef Scenario < handle
    
    
    properties
        % Define elements to be simulated
        vehicle % Instance of teh Vehicle class
        world % Instance of the Polygon world class
        
        % Simulation parameters
        plot_during_sim; % true => plot while simulating (requires euler integration)
        t0 = 0; % Initial time of simulation
        dt = 0.1; % Simulation step size
        tf = 10; % Final time of simulation
        
        % Simulation results
        tmat = [] % Matrix of time values
        xmat = [] % Matrix of state values
    end
    
    methods (Abstract)
        runScenario(obj); % Runs the Scenario
        u = control(obj, t, x); % Calculates the control to the vehicle
        plotState(obj, t); % Plots the current state of the vehicle
    end
    
    methods
        function obj = Scenario(vehicle,world,plot_during_sim)
            obj.vehicle = vehicle;
            obj.world = world;
            obj.plot_during_sim = plot_during_sim;
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
            obj.xmat = zeros(length(obj.vehicle.x), len);
            obj.xmat(:,1) = obj.vehicle.x;

            % Loop through and calculate the state
            for k = 1:len
                % Calculate state update equation
                t = obj.tmat(k);
                u = obj.control(t,obj.vehicle.x);
                xdot = obj.vehicle.kinematics.kinematics(t, obj.vehicle.x, u);

                % Update the state
                obj.vehicle.x = obj.vehicle.x + obj.dt * xdot;

                % Store the state
                obj.xmat(:,k) = obj.vehicle.x;
                
                % Plot the state
                if obj.plot_during_sim
                    obj.plotState(t);
                    pause(obj.dt/4); % dt/4 to allow for computation time as well
                end
            end
        end
        
        function ind = getStateIndex(obj, t)
            ind = (t-obj.t0)/obj.dt;
        end
    end
end

