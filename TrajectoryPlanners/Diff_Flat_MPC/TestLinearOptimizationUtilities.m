function TestLinearOptimizationUtilities
close all;

    %% Setup the parameters of the problem
    % Setup continuous dynamics
    Z = zeros(2); % 2x2 matrix of zeros
    I = eye(2); % 2x2 identity matrix
    A = [Z I Z Z; Z Z I Z; Z Z Z I; Z Z Z Z]; % state matrix
    B = [Z; Z; Z; I]; % Input matrix
    N = 100;
    dt = 0.05;
    
    % Setup the cost matrices
    cost_mat.Q = 10 .* diag([1, 1,  0, 0,   0, 0, 0, 0]); % state error squared (x-x_d)'Q(x-x_d)
    cost_mat.R = 0.1 .* diag([1, 1]); % Input squared cost (i.e. u'*R*u)
    cost_mat.S = []; % 10 .* diag([1, 1,  0, 0,   0, 0, 0, 0]);
    
    % Setup the desired trajectory
    traj = ConstantPosition(dt, 0, [5; 3], 3);
    %traj = OrbitTrajectory(dt, 0, [5; 3], 6, 1);
    
    %P = LinearSystemQuadraticCost(A, B, N, dt);
    P = LinearSystemQuadraticCostOSQP(A, B, N, dt, traj, cost_mat, [], [], []);
    P = P.initializeParameters();
    P.xd = P.calculateDesiredState(0); % Calculate the desired state and input
    P.ud = P.calculateDesiredInput(0);
    
    %% Set state and input bounds
    % Unconstrained
    x_max = [inf; inf; 10; 10; 10; 10; 10; 10];
    u_max = [inf; inf];
    
    % Some constraints
    %x_max = [10; 10; 0.5; 0.5; 1; 1; 5; 5];
    %u_max = [1; 1];
    
    % Better constraints
    %x_max = [inf; inf; 1; 1; 0.25; 0.25; 0.125; 0.125];
    %u_max = [1; 1];
    
    x_min = -x_max;
    P = P.updateSimBounds(x_min, x_max, u_max);    
    
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
    M = 600; % Number of MPC steps to take
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
            
            % Plot results
            [h_d, h_x] = P.plot2dPosition(x, ax, h_d, h_x);
            axis equal
            pause(0.0001);
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
    P.plotStateAndInput(xdata, udata);
end