function TestLinearOptimizationUtilities
close all;

    %% Setup the parameters of the problem
    % Setup continuous dynamics
    Z = zeros(2); % 2x2 matrix of zeros
    I = eye(2); % 2x2 identity matrix
    A = [Z I Z; Z Z I; Z Z Z]; % state matrix
    B = [Z; Z; I]; % Input matrix
    
    P = LOUParams(A, B);
    
    % Test the discrete dynamics
    LOU.testDiscreteDynamics(P);

    % Create the initial input and state
    u0 = 0.0.*ones(P.n_ctrl, 1);
    x0 = LOU.discreteSim(u0, P);
    
    % Create figure for plotting
    figure;
    ax = gca;
    h_d = [];
    h_x = [];
    
    % Optimize
    for k = 0:10000
        % Calculate desired state and inputs
        P.xd = LOU.calculateDesiredState(P, k);
        P.ud = LOU.calculateDesiredInput(P, k);
    
        % Optimize
        tic
        %[x, u] = LOU.simultaneousOptimization(x0, u0, P);
        [x, u] = LOU.sequentialOptimization(u0, P);
        time_opt = toc
        
        % Plot
        [h_d, h_x] = LOU.plot2dPosition(x, P, ax, h_d, h_x);
        pause(0.0001);
        
        % Update for the next iteration
        xf = x(end-P.n_x+1:end);
        u0 = [u(P.n_u+1:end); zeros(P.n_u, 1)];
        x0 = [x(P.n_x+1:end); P.Abar*xf];
        P.x0 = x0(1:P.n_x);
        
    end
    
    % Plot the results
    LOU.plotStateAndInput(x, u, P);
    
    
end