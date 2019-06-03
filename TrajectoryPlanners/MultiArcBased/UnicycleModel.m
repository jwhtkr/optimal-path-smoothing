classdef UnicycleModel < handle
    properties
        %% Select the gradient descent method
        cost;
        %step = @(u) cost.constant(u);
        %step = @(u) cost.diminish(u);
        %step = @(u) cost.optimal_step(u);
        step = @(u) cost.armijo_step(u);
        
    end
    
    methods
        
        function obj = UnicycleModel(x)
            %cost = Unicycle(x);
            cost = Unicycle2(x);
        end
        
        function u = minimize(obj,u)
            %% Initialize Cost and Plots
            %     cost.plotTraj(u);
            %     cost.plotCostVsIteration(u);
            %     cost.plotCost();
            %     cost.plotState(u);
            %     pause();
            
            %% Preform Gradient Descent
            i = 1;
            % TODO cost.stop update
            %while ~cost.stop(u) %i < 11
            while ~cost.armijo_stop(u)
                u = u + obj.step(u);
                %        i = i+1
                %
                %        cost.plotTraj(u);
                %        cost.plotCostVsIteration(u);
                %        cost.plotState(u);
                %        pause(0.1);
            end
            
            %     u_fmin = fmincon(@(u)cost.cost(u),u,A,b);
            %     display(u_fmin, 'Fmincon Velocities');
            
        end
    end
end

