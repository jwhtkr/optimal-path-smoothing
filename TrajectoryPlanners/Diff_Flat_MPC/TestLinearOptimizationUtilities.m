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
    
    %P = LinearSystemQuadraticCost(A, B, N, dt);
    P = LinearSystemQuadraticCostOSQP(A, B, N, dt, [], [], []);
    P = P.initializeParameters();
    
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
        % Calculate desired state and inputs
        P.xd = P.calculateDesiredState(k);
        P.ud = P.calculateDesiredInput(k);
    
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
        
        % Update for the next iteration
        xf = x(end-P.n_x+1:end);
        u0 = [u(P.n_u+1:end); zeros(P.n_u, 1)];
        x0 = [x(P.n_x+1:end); P.Abar*xf];
        P = P.setInitialState(x0(1:P.n_x));
    end
    
    % Plot the results
    P.plotStateAndInput(xdata, udata);
end