classdef MPCG2GAgent < SingleAgent
    %MPCG2GAgent Creates an agent that will use differentially flat MPC
    %approach to move towards a goal
    
    properties(SetAccess = protected)
        xd % desired end state
        solver % Linear system solver
        u_warm % The initial input
        x_warm % The initial state
        x_flat_latest % The latest differentially flat to follow
        zero_error_tracking = true; % Uses zero-error epsilon control for tracking if true, otherwise uses standard tracking
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
            
            % Initialize epsilon control
            obj.vehicle.use_dim_eps = false; % Do not have epsilon diminish
            obj.vehicle.eps_path = 0.2;
            
            % Create the continuous time linear system
            Z = zeros(2); % 2x2 matrix of zeros
            I = eye(2); % 2x2 identity matrix
            A = [Z I Z Z; Z Z I Z; Z Z Z I; Z Z Z Z]; % state matrix
            B = [Z; Z; Z; I]; % Input matrix           
            
            % Setup the desired trajectory
            %traj = ConstantPosition(MultiAgentScenario.dt, 0, qd, 3);
            traj = OrbitTrajectory(MultiAgentScenario.dt, 0, qd, 6, 1);
            
            % Create MPC solver
            obj.solver = LinearSystemQuadraticCostOSQP(A, B, 300, MultiAgentScenario.dt, traj, [], [], []);
            obj.solver = obj.solver.initializeParameters();
            %obj.solver.xd = obj.solver.calculateDesiredState(0); % Calculate the desired state and input
            %obj.solver.ud = obj.solver.calculateDesiredInput(0);
            
            % Set state and input bounds
            %x_max = [inf; inf; 0.5; 0.5; 1; 1; 5; 5];
            %x_max = [inf; inf; 1; 1; 1; 1; 0.5; 0.5];
            x_max = [inf; inf; 1; 1; 0.25; 0.25; 0.125; 0.125];
            x_min = -x_max;
            u_max = [1.0; 1.0];
            obj.solver = obj.solver.updateSimBounds(x_min, x_max, u_max);
            
            % Set initial state of the MPC
            obj.x_flat_latest = [obj.getFlatStateFromVehicleState(x0, [0;0]); 0; 0];
            %TODO: remove next line
            obj.x_flat_latest(1:2) = obj.x_flat_latest(1:2) + rand(2,1)*1;
            obj.solver = obj.solver.setInitialState(obj.x_flat_latest);
            
            % Create the initial input and state for warm starting
            obj.u_warm = 0.1.*ones(obj.solver.n_ctrl, 1);
            obj.x_warm = obj.solver.discreteSim(obj.u_warm);
            
            if abs(x0(BetterUnicycleVehicle.v_ind)) < 0.001
                warning('This class will have issues with zero velocity initial conditions');
            end
        end
        
        %%%%  Superclass Method Implementation %%%%
        function u = control(obj, t, x_state, ~)
            % Set the desired states in the MPC for differentially flat
            % state
            k = MultiAgentScenario.convertTimeToStep(t);
            obj.solver = obj.solver.updateDesiredTrajectory(k);
            %obj.solver.xd = obj.solver.calculateDesiredState(k);
            %obj.solver.ud = obj.solver.calculateDesiredInput(k);
            
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
            obj.solver = obj.solver.setInitialState(obj.x_warm(1:obj.solver.n_x)); % Update the initial state
            
            if obj.zero_error_tracking
                % Calculate the positions desired states for the epsilon point
                [qe, qe_dot, qe_ddot] = getDesiredEspilonPoints(obj.x_flat_latest, obj.vehicle.eps_path, 0.1);

                % Calculate the epsilon tracking control
                u = obj.vehicle.pathControl(t, qe, qe_dot, qe_ddot, x_state);
            else
                % Extract the desired values
                qd = obj.x_flat_latest(1:2);
                qd_dot = obj.x_flat_latest(3:4);
                qd_ddot = obj.x_flat_latest(5:6);
                
                % Calculate the control
                u = obj.vehicle.pathControl(t, qd, qd_dot, qd_ddot, x_state);
            end
            
            
%             % Store the state for exact representation
%             rep = ExactTrajRep4();
%             rep.setValues(x, u, obj.solver.dt, t);
%             rep.plotComparison();

%             % Test diff flat LQR P matrix generation
%             diffLQR = DiffFlatLQR();
%             diffLQR.setValues(x, u, obj.solver.dt, t);
%             u = diffLQR.calculateControl(t, x_state);
            

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
            [~, ~, ~, a, alpha] = TrajUtil.getTrajectoryInformation(traj);
            
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

function [qe, qe_dot, qe_ddot] = getDesiredEspilonPoints(x_flat, eps, thresh)
%getDesiredEspilonPoints will calculate the desired epsilon movement based
%on the information contained in x_flat
%
% Note that this does not perform well when qd_dot is too small. When v is
% below the defined threshold then qe_dot and qe_ddot are set to zero
%
% Inputs:
%   x_flat: 8x1 vector where x_flat = [qd; qd_dot; qd_ddot; qd_dddot]
%   eps: the distance in front of qd to find the desired epsilon point
%   thresh: threshold for switching to pure position control
%
% Outputs:
%   qe: desired epsilon point position
%   qe_dot: desired epsilon point velocity vector
%   qe_ddot: desired epsilon point acceleration vector

    % Extract the trajectory information from x_flat
    traj.q = x_flat(1:2);
    traj.qdot = x_flat(3:4);
    traj.qddot = x_flat(5:6);
    traj.qdddot = x_flat(7:8);
    
    % Calculate the trajectory parameters
    [psi, v, w, a, alpha] = TrajUtil.getTrajectoryInformation(traj);
    
    % Process and group the states
    c = cos(psi); % Pre-calculate the trig functions
    s = sin(psi);
    vbar = [v; w]; % Velocities
    abar = [a; alpha]; % Accelerations    
    
    % Calculate the desired epsilon point
    qe = traj.q + eps*[c; s];
    
    % Calculate the desired epsilon point velocity vector
    Re = [c -eps*s; s eps*c];
    qe_dot = Re * vbar;
    
    % Calculate the desired epsilon point acceleration vector
    w_hat_e = [0 -eps*w; w/eps 0];
    qe_ddot = Re*w_hat_e *vbar + Re*abar;
    
    % Threshold everything if v is small
    if v <= thresh
        qe_dot = [0;0];
        qe_ddot = [0;0];
    end
end


