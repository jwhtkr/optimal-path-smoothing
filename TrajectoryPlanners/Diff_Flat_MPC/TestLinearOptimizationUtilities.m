function TestLinearOptimizationUtilities
close all;

    %% Setup the parameters of the problem
    % Setup continuous dynamics
    Z = zeros(2); % 2x2 matrix of zeros
    I = eye(2); % 2x2 identity matrix
    A = [Z I Z; Z Z I; Z Z Z]; % state matrix
    B = [Z; Z; I]; % Input matrix
    
    P = LinearSystemQuadraticCost(A, B, 50);
    
    % Test the discrete dynamics
    P.testDiscreteDynamics();

    % Create the initial input and state
    u0 = 0.0.*ones(P.n_ctrl, 1);
    x0 = P.discreteSim(u0);
    
    % Create figure for plotting
    figure;
    ax = gca;
    h_d = [];
    h_x = [];
    
    % Optimize
    for k = 0:100000
        % Calculate desired state and inputs
        P.xd = P.calculateDesiredState(k);
        P.ud = P.calculateDesiredInput(k);
    
        % Optimize
        %tic
        %[x, u] = P.simultaneousOptimization(x0, u0);
        [x, u] = P.sequentialOptimization(u0);
        %time_opt = toc
        
        % Plot
        [h_d, h_x] = P.plot2dPosition(x, ax, h_d, h_x);
        pause(0.0001);
        
        % Update for the next iteration
        xf = x(end-P.n_x+1:end);
        u0 = [u(P.n_u+1:end); zeros(P.n_u, 1)];
        x0 = [x(P.n_x+1:end); P.Abar*xf];
        P.x0 = x0(1:P.n_x);        
    end
    
    % Plot the results
    P.plotStateAndInput(x, u);
end