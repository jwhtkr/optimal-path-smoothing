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
    
    % Setup the desired trajectory
    traj = ConstantPosition(0, dt, [5; 3], 3);
    
    %P = LinearSystemQuadraticCost(A, B, N, dt);
    P = LinearSystemQuadraticCostOSQP(A, B, N, dt, traj, [], [], []);
    P = P.initializeParameters();
    P.xd = P.calculateDesiredState(0); % Calculate the desired state and input
    P.ud = P.calculateDesiredInput(0);
    
    % Set state and input bounds
    x_max = [10; 10; 0.5; 0.5; 1; 1; 5; 5];
    x_min = -x_max;
    u_max = [1.0; 1.0];
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
    
    % Optimize
    for k = 1:M
        k
        % Optimize
        tic
        [x, u] = P.simultaneousOptimization(x0, u0);
        %[x, u] = P.sequentialOptimization(u0);
        time_opt = toc
        
        % Store the updated data
        xdata(:,k) = P.x0;
        udata(:,k) = u(1:P.n_u);
        
        % Plot
        [h_d, h_x] = P.plot2dPosition(x, ax, h_d, h_x);
        pause(0.0001);
        
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