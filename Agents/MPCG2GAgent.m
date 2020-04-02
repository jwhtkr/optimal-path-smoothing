classdef MPCG2GAgent < SingleAgent
    %MPCG2GAgent Creates an agent that will use differentially flat MPC
    %approach to move towards a goal
    
    properties(SetAccess = protected)
        xd % desired end state
        solver % Linear system solver
        u_warm % The initial input
        x_warm % The initial state
        x_flat_latest % The latest differentially flat to follow
    end
    
    properties (Constant)
        % Sensing variables
        n_sensors = 30; % Number of sensor lines
        max_sensor_range = 4; % Maximum range of the sensor
    end
    
    
    methods
        function obj = MPCG2GAgent(x0, qd, world)
            % initialize the scenario
            obj = obj@SingleAgent(BetterUnicycleVehicle(x0), world);
            obj.vehicle.sensor.initializeSensor(obj.n_sensors, obj.max_sensor_range);
            
            % Store the desired end state
            xd = [qd; zeros(3,1)];
            
            % Create the continuous time linear system
            Z = zeros(2); % 2x2 matrix of zeros
            I = eye(2); % 2x2 identity matrix
            A = [Z I Z; Z Z I; Z Z Z]; % state matrix
            B = [Z; Z; I]; % Input matrix            
            
            % Create MPC solver
            obj.solver = LinearSystemQuadraticCostOSQP(A, B, 100, MultiAgentScenario.dt, [], [], []);
            obj.solver = obj.solver.initializeParameters();
            
            % Set initial state of the MPC
            obj.x_flat_latest = obj.getFlatStateFromVehicleState(x0, [0;0]);
            obj.solver = obj.solver.setInitialState(obj.x_flat_latest);
            
            % Create the initial input and state for warm starting
            obj.u_warm = 0.1.*ones(obj.solver.n_ctrl, 1);
            obj.x_warm = obj.solver.discreteSim(obj.u_warm);
            
            if abs(x0(BetterUnicycleVehicle.v_ind)) < 0.001
                warning('This class will have issues with zero velocity initial conditions');
            end
        end
        
        %%%%  Superclass Method Implementation %%%%
        function u = control(obj, t, x, ~)
            % Set the desired states in the MPC for differentially flat
            % state
            k = MultiAgentScenario.convertTimeToStep(t);
            obj.solver.xd = obj.solver.calculateDesiredState(k);
            obj.solver.ud = obj.solver.calculateDesiredInput(k);
            
            % Perform MPC optimization for differentially flat state
            tic
            [x, u] = obj.solver.simultaneousOptimization(obj.x_warm, obj.u_warm);
            toc
            
            % Extract the desired state
            obj.x_flat_latest = x(1:obj.solver.n_x);
            
            % Update for the next iteration
            xf = x(end-obj.solver.n_x+1:end);
            obj.u_warm = [u(obj.solver.n_u+1:end); zeros(obj.solver.n_u, 1)]; % warm-start for next iteration is simply the zero input control appended to previous horizon
            obj.x_warm = [x(obj.solver.n_x+1:end); obj.solver.Abar*xf];
            obj.solver = obj.solver.setInitialState(obj.x_warm(1:obj.solver.n_x));    
            
            % Calculate the current state in the differentially flat
            % coordinate system
            
            % Update the initial state
            
            % Calculate the MPC
            
            % Calculate the first control
            
            % MPC here
            u = [0;0];
        end
    end
    
    % Methods for converting to and from differentially flat system
    methods
        function x_flat = getFlatStateFromVehicleState(obj, x_veh, u_veh)
        % getFlatStateFromVehicleState this function will return the flat
        % state given the vehicle state
        %
        % Inputs:
        %   x_veh: vehicle state defined by a BetterUnicycle Model
        %
        % Outputs:
        %   x_flat = [q; qdot; qddot]
        
            % Get position
            x_flat = zeros(6,1);
            x_flat(1:2) = x_veh(obj.vehicle.q_ind);
            
            % Get velocity
            v = x_veh(obj.vehicle.v_ind);
            th = x_veh(obj.vehicle.th_ind);
            h = [cos(th); sin(th)];
            x_flat(3:4) = v * h;
            
            % Get acceleration
            a = u_veh(obj.vehicle.u_a_ind);
            w = x_veh(obj.vehicle.w_ind);
            x_flat(5:6) = a*h + v*w*[0 -1; 1 0] * h;
        end
        
        function ud = getInputFromFlatState(obj, x_flat, u_flat)
        %getInputFromFlatState calculates the input given a flat state
        %
        % Inputs:
        %   x_flat = [q; qdot; qddot]
        %   u_flat = qdddot
        %
        % Outputs:
        %   ud = [a; alpha], the translational and rotational acceleration
        %   commands
        
            % Place flat trajectory into a trajectory struct
            traj.q = x_flat(1:2);
            traj.qdot = x_flat(3:4);
            traj.qddot = x_flat(5:6);
            traj.qdddot = u_flat;
            
            % Extract the desired accelerations
            [~, ~, ~, a, alpha] = getTrajectoryInformation(traj);
            
            % Output the control
            ud = [a; alpha];            
        end
    end
    
    % Methods for accessing the latest differentially flat data
    methods
        function [x_mat, u_mat] = getLatestMPCResults(obj)
            [x_mat, u_mat] = obj.solver.getInputStateMatrices(obj.x_warm, obj.u_warm);
        end
        
        function q_mat = getLatestTrajectory(obj)
            [x_mat, ~] = obj.getLatestMPCResults();
            q_mat = x_mat(1:2, :);
        end
        
        function q_latest = getLatestTrajectoryPoint(obj)
            q_latest = obj.x_flat_latest(1:2);
        end
    end
end


    function [psi, v, w, a, alpha] = getTrajectoryInformation(traj)
    %getTrajectoryInformation calcualte trajectory information directly from
    %trajectory
    %
    % Inputs:
    %   traj: Struct with trajectory information
    %       .q = position
    %       .qdot = velocity vector
    %       .qddot = acceleration vector
    %       .qdddot = jerk vector
    %
    % Outputs:
    %   psi: orientation
    %   v: translational velocity
    %   w: rotational velocity
    %   a: translational acceleration
    %   alpha: rotational acceleration

        % Extract trajectory information
        xdot = traj.qdot(1); % Velocity vector
        ydot = traj.qdot(2);
        xddot = traj.qddot(1); % Accleration vector
        yddot = traj.qddot(2);
        xdddot = traj.qdddot(1); % Jerk vector
        ydddot = traj.qdddot(2);

        % Calculate the trajectgory variables
        psi = atan2(ydot, xdot);
        v = sqrt(xdot^2+ydot^2);
        w = 1/v^2*(xdot*yddot - ydot*xddot);
        a = (xdot*xddot + ydot*yddot)/v;
        alpha = (xdot*ydddot-ydot*xdddot)/v^2 - 2*a*w/v;    
    end