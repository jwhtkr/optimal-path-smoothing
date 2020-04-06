classdef TrajUtil
    %TrajUtil provides static functions for working on trajectory data
    
    methods (Static)
        function traj = createClothoidTrajectory(waypoints, velocity, time_step, max_curve, delta_curve )
            %createClothoidTrajectory Will create a clothoid trajectory based on the
            %waypoints and return a 2D trajectory
            %
            % Inputs:
            %   waypoints: an array of path points in the desired order (e.g., [q0,q1,...,qn] 
            %              where qi = [x; y]
            %   velocity: Desired velocity of the trajectory
            %   time_step: Discrete time step in the trajectory
            %   max_curve: Maximum curvature possible for the vehicle
            %   delta_curve: Maximum change in curvature for the vehicle
            %
            % Outputs:
            %   traj: Continuous curvature trajectory stored in an instance of the
            %         Trajectory2D object

                traj = CCPathGenerator(waypoints, velocity, time_step, max_curve, delta_curve).traj;
        end
        
        function traj = createOffsetTrajectory(traj_in, q_off)
        %createOffsetTrajectory Takes in a trajectory and outputs a trajectory
        %which is offset by the offset vector
        %
        % Inputs:
        %   traj_in: Instance of the Trajectory2D class
        %   q_off: 2x1 offset vector
        %
        % Outputs:
        %   traj: Resulting trajectory, instance of Trajectory2D class

            % Extract variables for easy access
            q_leader = [traj_in.x; traj_in.y];
            qdot_leader = [traj_in.xdot; traj_in.ydot];
            qddot_leader = [traj_in.xddot; traj_in.yddot];
            qdddot_leader = [traj_in.xdddot; traj_in.ydddot];

            % Initialize storage variables for new trajectory
            len = size(q_leader, 2);
            q_follower = zeros(2, len);
            qdot_follower = zeros(2, len);
            qddot_follower = zeros(2, len);
            qdddot_follower = zeros(2, len);        

            % Loop through all trajectory points and calculate a new trajectory
            for k = 1:length(traj_in.psi)
                R = [cos(traj_in.psi(k)), -sin(traj_in.psi(k)); sin(traj_in.psi(k)), cos(traj_in.psi(k))];
                w_hat = [0 -traj_in.w(k); traj_in.w(k) 0];
                alpha_hat = [0 -traj_in.alpha(k); traj_in.alpha(k) 0];
                zeta_hat = [0 -traj_in.zeta(k); traj_in.zeta(k) 0];

                q_follower(:,k) = R*q_off + q_leader(:,k);
                qdot_follower(:,k) = R*w_hat*q_off + qdot_leader(:,k);
                qddot_follower(:,k) = R*(w_hat^2 + alpha_hat)*q_off + qddot_leader(:,k);
                qdddot_follower(:,k) = R*(w_hat^3 + zeta_hat + 3*w_hat*alpha_hat)*q_off + qdddot_leader(:,k);
            end

            % Store the trajectory data into a Trajectory2D class
            traj = Trajectory2D();
            traj.x = q_follower(1,:);
            traj.y = q_follower(2,:);
            traj.xdot = qdot_follower(1,:);
            traj.ydot = qdot_follower(2,:);
            traj.xddot = qddot_follower(1,:);
            traj.yddot = qddot_follower(2,:);
            traj.xdddot = qdddot_follower(1,:);
            traj.ydddot = qdddot_follower(2,:);
            traj.xddddot = zeros(1,length(traj.x));
            traj.yddddot = zeros(1,length(traj.y));
            traj.dt = traj_in.dt;
            traj.updateTrajWithPositionalValues();    
        end
    end
end

