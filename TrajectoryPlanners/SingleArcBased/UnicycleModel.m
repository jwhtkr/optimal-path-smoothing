function UnicycleModel
    close all;
    
    rmpath ../UnicycleMultipleTimeSteps/Opt_dep
    % Add dependent folders
    addpath Opt_dep
    
    %% Initial Input and State
    u = [0 0]'; % only one time step 
    %u = [0.15; 0.01];
    x = [0 0 pi/5 0 0]';
    A = [];
    b = [];
    
    %% Create the cost
    %cost = Unicycle(x);
    cost = Unicycle2(x);
    
    %% Select the gradient descent method
    %step = @(u) cost.constant(u);
    %step = @(u) cost.diminish(u);
    %step = @(u) cost.optimal_step(u);
    step = @(u) cost.armijo_step(u);
    
    %% Initialize Cost and Plots
    cost.plotTraj(u);
    cost.plotCostVsIteration(u);
%     cost.plotCost();
%     cost.plotState(u);
    pause();
    
    %% Preform Gradient Descent
    i = 1;
    % TODO cost.stop update  
    %while ~cost.stop(u) %i < 11
    while ~cost.armijo_stop(u) %i < 11
       u = u + step(u);
       i = i+1
       
       cost.plotTraj(u);
       cost.plotCostVsIteration(u);
%        cost.plotState(u);
       pause(0.1);
    end
    display(u, 'Parameter Opt Velocities');
    display(cost.cost(u), 'Final cost');
    
    %% Fmincon validation
    u_fmin = fmincon(@(u)cost.cost(u),u,A,b);
    display(u_fmin, 'Fmincon Velocities');

end

