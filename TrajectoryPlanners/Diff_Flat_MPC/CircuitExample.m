function CircuitExample
close all;

    %% Setup the parameters of the problem
    % Setup continuous dynamics
    R = 1; L = 2; C = 3;
    A = [-R/L -1/L; 1/C 0]; % State matrix
    B = [1/L; 0]; % Input matrix
    N = 100;
    dt = 0.05;
    tf_sim = 20;
    x0 = [50; 0];
    
    % Setup the desired trajectory
    xd = [0;4];
    traj = ConstantState(dt, 0, xd, xd(2));
    
    % Setup the cost matrices
    cost_params.Q = eye(2);
    cost_params.R = 1;
    cost_params.S = []; 
    
%     % Setup the cost matrices - Bryson's method
%     cost_params.Q = diag([1/25 1/25]); % No state greater than 5
%     cost_params.R = 1/16; % No input greater than 4
%     cost_params.S = []; 
    
    %% Set state and input bounds
    % Unconstrained
    x_max = [inf; inf];
    x_min = -x_max;
    u_max = [inf];
    
    % Some constraints
    x_max = [inf; inf];
    x_min = [-inf; -inf];
    u_max = [4];
    
    cost_params.x_max = x_max;
    cost_params.x_min = x_min;
    cost_params.u_max = u_max;
    
    
    % Setup MPC problem
    P = LinearSystemQuadraticCostOSQP(A, B, N, dt, traj, cost_params, [], [], []);
    P = P.initializeParameters();
    P = P.setInitialState(x0);
    P.xd = P.calculateDesiredState(0); % Calculate the desired state and input
    P.ud = P.calculateDesiredInput(0);
      
    
    % Test the discrete dynamics
    P.testDiscreteDynamics();

    % Create the initial input and state
    u0 = 1.0.*ones(P.n_ctrl, 1);
    x0 = P.discreteSim(u0);
    
    % Create figure for plotting
    figure;
    ax = gca;
    h_d = [];
    h_x = [];
    
    % Data for MPC
    M = round(tf_sim/dt);
    xdata = zeros(P.n_x, M);
    udata = zeros(P.n_u, M);
    plot_step = 10;
    
    % Optimize
    for k = 1:M
        % Optimize
        %tic
        [x, u] = P.simultaneousOptimization(x0, u0);
        %[x, u] = P.sequentialOptimization(u0);
        %time_opt = toc
        
        % Store the updated data
        xdata(:,k) = P.x0;
        udata(:,k) = u(1:P.n_u);
        
        % Plot
        if mod(k, plot_step) == 0
            % Display step       
            k
            
%             % Plot results
%             [h_d, h_x] = P.plot2dPosition(x, ax, h_d, h_x);
%             axis equal
%             pause(0.0001);
        end
        
        %% Update for the next iteration
        % Create a warm start
        xf = x(end-P.n_x+1:end); % Create a warm start for the next iteration
        u0 = [u(P.n_u+1:end); zeros(P.n_u, 1)];
        x0 = [x(P.n_x+1:end); P.Abar*xf];
        
        % Set the initial state
        P = P.setInitialState(x0(1:P.n_x));
        
        % Update the desired trajectory
        P = P.updateDesiredTrajectory(k); % Set k because the step is zero indexed, thus k = step+1
    end
    
    % Plot the results
    P.plotStateAndInputStacked(xdata, udata);
end