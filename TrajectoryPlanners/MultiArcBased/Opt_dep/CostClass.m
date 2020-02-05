classdef (Abstract) CostClass < handle
    
    % Plotting properties
    properties
        x1lim % Limits for plotting along x1 axis
        x2lim % Limits for plotting along x2 axis
        zlim % Limits for plotting vertical axis
        dx = 0.005
        n % number of states
        
        % Plotting handles
        cost_vs_iter_handle = [];  % Handle to cost vs iteration plot
        surf_contour_fig = [];  % Handle to the figure for surf and contour plots
    end
    
    % Gradient descent variables
    properties
        x0 % Initial state
        eps = 0.1; % Stopping criteria
        armijo_eps = .0001; % Armijo stopping criteria
        iter_num = 1 % stores the number of times the gradient has been called
        
        % Stepsizes
        alpha_const = 0.001; % Stepsize for constant steps
        alpha_dim = 0.0005; % Stepsize for diminishing steps
        
        % Armijo variables
        alpha = 0.5;
        beta = 0.5;
        k = 10;
    end
    
    % Cost function methods
    methods(Abstract)
        c = cost(obj, x)
        dc_dx = partial(obj, x)
        step = optimal_step(obj, x)
    end
    
    methods
        function obj = CostClass(x1lim, x2lim, zlim, x0)
            % Read in properties
            obj.x1lim = x1lim;
            obj.x2lim = x2lim;
            obj.zlim = zlim;
            obj.x0 = x0;
            obj.n = length(x0);            
        end
        
        %%%%%%%%%  Evaluation functions %%%%%%%%%%%%
        function numericallyTestLocalSolution(obj, x, dx, max_dev)
        % This function will numerically test points around a solution
        % deviating around x by max_dev with a resolution of dx
        
            % Check for two dimensions
            if length(x) ~= 2
                return;
            end
            
            % Get data points to test 
            x1_vec = [x(1)-max_dev:dx:x(1)+max_dev];
            x2_vec = [x(2)-max_dev:dx:x(2)+max_dev];
            [X1, X2] = meshgrid(x1_vec, x2_vec);
            
            % Initialize minimum value as the input value
            x_min = x;  % Initialize the smallest value for x as the input
            c_min = obj.cost(x); % Initialize the smallest cost in terms of the input
            disp(['Input x = ', obj.formatArray(x), ' With cost = ', num2str(c_min)]);
            
            % Search for a smaller costing x
            found_smaller = false;
            for m = 1:size(X1, 1)
                for n = 1:size(X1, 1)
                    % Create the vector to evaluate
                    x_eval = [X1(m,n); X2(m,n)];
                    
                    % Check to see if the value satisifies the constraints
                    if ~obj.satisfiesConstraints(x_eval)
                        continue;
                    end
                    
                    % Evaluate the cost
                    c_eval = obj.cost(x_eval);
                    
                    % Compare the cost to the smallest value seen
                    if c_eval < c_min
                        % Store smaller values
                        found_smaller = true;
                        x_min = x_eval;
                        c_min = c_eval;
                    end
                end
            end
            
            % Output results
            if found_smaller
                disp(['Found a smaller result: ', obj.formatArray(x_min), ' With cost = ', num2str(c_min)]);
            else
                disp('No smaller cost found');
            end
        end
        
        function result = satisfiesConstraints(obj, x)
            % Returns true if the input satisfies constraints
            result = true; 
        end
        
        function text = formatArray(obj, x)
            text = ['[', num2str(x(1)), ', ' num2str(x(2)), ']'];
        end
        
        %%%%%%%%%  Plotting functions %%%%%%%%%%%%
        function plotCost(obj, varargin)
            if obj.n ~= 2 % Only plot for 2D optimization
                return;
            end
            
            % Calculate values to be evaluated
            x1_vec = [obj.x1lim(1):obj.dx:obj.x1lim(2)];
            x2_vec = [obj.x2lim(1):obj.dx:obj.x2lim(2)];
            [X1, X2] = meshgrid(x1_vec, x2_vec);
            
            % Get cost function to be used
            if nargin < 2
                costfunction = @(x)obj.cost(x);
            else
                costfunction = @(x)varargin{1}(x);
            end
            
            % Loop through and evaluate each value
            Z = zeros(size(X1));
            for m = 1:size(X1,1)
                for n = 1:size(X1,2)
                    x = [X1(m,n); X2(m,n)];
                    Z(m,n) = min(obj.zlim(2), costfunction(x));
                end
            end
            
            % Plot the surf and contour plots
            if isempty(obj.surf_contour_fig)
                obj.surf_contour_fig = figure;
            end
            subplot(1,2,1);
            surf(X1,X2,Z)
            zlim(obj.zlim);            
            colormap(pink);
            shading interp
            
            subplot(1,2,2);
            contour(X1,X2, Z, 60);
            hold on;
        end  
        
        function plotState(obj, x)
            if obj.n ~= 2 % Only plot for 2D optimization
                return;
            end
            
            set(0, 'currentfigure', obj.surf_contour_fig);
            subplot(1,2,2);
            plot(x(1), x(2), 'ko', 'linewidth', 3); 
            xlim(obj.x1lim);
            ylim(obj.x2lim);
            axis equal
        end
        
        function plotCostVsIteration(obj, x)
            % Create the figure
            if isempty(obj.cost_vs_iter_handle)
                figure;
                
                % get data to plot
                i = 0;  % iteration
                c = obj.cost(x); % cost
                
                % plot data
                obj.cost_vs_iter_handle = plot(i, c, 'ro', 'linewidth', 3);
                xlabel('Iteration', 'fontsize', 18);
                ylabel('Cost', 'fontsize', 18);
            else
                % Extract data from plot
                iterations = get(obj.cost_vs_iter_handle, 'xdata');
                costs = get(obj.cost_vs_iter_handle, 'ydata');
                
                % Create new data
                i = iterations(end) + 1;
                c = obj.cost(x);
                
                % Set new data
                set(obj.cost_vs_iter_handle, 'xdata', [iterations i], 'ydata', [costs c]);                
            end
            
        end
        
        %%%%%%%%%  Gradient Step Calculations %%%%%%%%%%%%
        function dc_dx = numericPartial(obj, x)
            dc_dx = zeros(1, length(x));
            
            % Vary each value independently
            delta = .001;
            c = obj.cost(x);
            for i = 1:length(x)
                % Vary the ith value of k
                xhat = x;
                xhat(i) = xhat(i) + delta;
                
                % Calculate the corresponding cost for the varied value
                chat = obj.cost(xhat);
                
                % Estimate the partial using the difference
                dc_dx(i) = (1/delta) * (chat - c);
            end
        end
        
        function bool = stop(obj, x)
            dc_dx = obj.partial(x);
            val = norm(dc_dx);
            if val < obj.eps
                bool = true;
            else
                bool = false;
            end
        end
        
        function step = constant(obj, x)
            dc_dx = obj.partial(x);
            step = -obj.alpha_const .* dc_dx';
        end
        
        function step = diminish(obj, x)
            dc_dx = obj.partial(x);
            step = -obj.alpha_dim / sqrt(obj.iter_num) .* dc_dx';
            obj.iter_num = obj.iter_num + 1;
        end
        
        function step = armijo_step(obj, x)
            dc_dx = obj.partial(x);
            
            % Perform an armijo search
%             obj.k = obj.k;
            obj.armijo_search(x, dc_dx);
            
            if length(x) > 3 %exist obj.ind_time1 && exist obj.ind_time2 && obj.ind_time3
                dtau1 = -obj.beta^obj.k * dc_dx(obj.ind_time1);
                dtau2 = -obj.beta^obj.k * dc_dx(obj.ind_time2);
                dtau3 = -obj.beta^obj.k * dc_dx(obj.ind_time3);
                
                if obj.T <= x(obj.ind_time3) + dtau3 && ~(obj.beta^obj.k == 0)
                    dc_dx(obj.ind_time3) = (obj.T - obj.dt - x(obj.ind_time3))/(-obj.beta^obj.k);
                    dtau3 = obj.T - obj.dt - x(obj.ind_time3);
                end
                if x(obj.ind_time3) + dtau3 < x(obj.ind_time2) + dtau2 && ~(obj.beta^obj.k == 0)
                    dc_dx(obj.ind_time2) = (x(obj.ind_time3) + dtau3 - x(obj.ind_time2))/(-obj.beta^obj.k);
                    dtau2 = x(obj.ind_time3) + dtau3 - x(obj.ind_time2);
                end
                if x(obj.ind_time2) + dtau2 < x(obj.ind_time1) + dtau1 && ~(obj.beta^obj.k == 0) 
                    dc_dx(obj.ind_time1) = (x(obj.ind_time2) + dtau2 - x(obj.ind_time1))/(-obj.beta^obj.k);
                    dtau1 = (x(obj.ind_time2) + dtau2 - x(obj.ind_time1));
                end
                if x(obj.ind_time1) + dtau1 < 0 && ~(obj.beta^obj.k == 0)
                    dc_dx(obj.ind_time1) = -x(obj.ind_time1)/(-obj.beta^obj.k);
                    dtau1 = -x(obj.ind_time1);
                end
                if x(obj.ind_time2) + dtau2 < x(obj.ind_time1) + dtau1 && ~(obj.beta^obj.k == 0) 
                    dc_dx(obj.ind_time2) = (x(obj.ind_time1) + dtau1 - x(obj.ind_time2))/(-obj.beta^obj.k);
                    dtau2 = (x(obj.ind_time1) + dtau1 - x(obj.ind_time2));
                end
                if x(obj.ind_time3) + dtau3 < x(obj.ind_time2) + dtau2 && ~(obj.beta^obj.k == 0)
                    dc_dx(obj.ind_time3) = (x(obj.ind_time2) + dtau2 - x(obj.ind_time3))/(-obj.beta^obj.k);
                    dtau3 = x(obj.ind_time2) + dtau2 - x(obj.ind_time3);
                end
                
                
                
            end
            % Output step
            step = -obj.beta^obj.k * dc_dx';
        end
        
        function result = armijo_stop(obj, x)
            dc_dx = obj.partial(x);
            c = obj.cost(x); % cost at x
            
            dc_squared = dc_dx * dc_dx';
            [cond1, cond2] = getArmijoConditions(obj, dc_dx, dc_squared, c, x);
            
            if (~cond1 && ~cond2) 
                result = true;
            %elseif abs(ck - ck1) < obj.eps
            elseif norm(obj.beta^obj.k * dc_dx') < obj.armijo_eps
                result = true;
                obj.k = 1;
            else
                result = false;
            end
        end
        
        function [cond1, cond2, ck, ck1] = getArmijoConditions(obj, dc_dx, dc_squared, c, x)
            xk = x - obj.beta^obj.k * dc_dx';       % x_k
            xk1 = x - obj.beta^(obj.k-1) * dc_dx';  % x_{k-1}
            ck = obj.cost(xk);                      % c(x_k)
            ck1 = obj.cost(xk1);                    % c(x_k-1)

            % Check condition 1
            if ck - c <= -obj.beta^obj.k * obj.alpha * dc_squared
                cond1 = true;
            else
                cond1 = false;
            end

            % Check condition 2
            if ck1 - c > -obj.beta^(obj.k-1) * obj.alpha * dc_squared
                cond2 = true;
            else
                cond2 = false;
            end
            
            % Check for inf or nan values (collision) and decrease the step
            % size
            sum = ck + ck1;
            if isnan(sum) || isinf(sum)
                cond1 = false;
                cond2 = true;                
            end
        end
        
        function armijo_search(obj, x, dc_dx)
            % Initialize variables
            dc_squared = dc_dx * dc_dx';
            c = obj.cost(x); % cost at x
            
            cond1 = false;
            cond2 = false;
            
            increase = false;
            decrease = false;
            
            % Store min cost and k found
            c_min = c;
            k_min = inf;
            
            % Perform search            
            while ~cond1 || ~cond2

                [cond1, cond2, ck, ck1] = getArmijoConditions(obj, dc_dx, dc_squared, c, x);
                
                % Store the min k and cost found
                if ck < c_min
                    c_min = ck;
                    k_min = obj.k;
                end
                if ck1 < c_min
                    c_min = ck1;
                    k_min = obj.k-1;
                end
                
                % Update k
                if cond1 && ~cond2
                    % Check to see if previous was increase
                    if increase
                        break;
                    end                    
                    
                    obj.k = obj.k-1;
                    decrease = true;
                    increase = false;
                elseif cond2 && ~cond1
                    % Check to see if previous was decrease
                    if decrease
                        break;
                    end
                    
                    obj.k = obj.k+1; 
                    decrease = false;
                    increase = true;
%                 elseif ~cond1 && ~cond2
%                     k = obj.k % dummy value to see k easier 
%                     break;
%                     
%                     if false
%                         subplot(1,2,2);
%                         hold on;
%                         axis equal
%                         xk = x - obj.beta^obj.k * dc_dx';       % x_k
%                         xk1 = x - obj.beta^(obj.k-1) * dc_dx';  % x_{k-1}
%                         plot(x(1), x(2), 'ro', 'linewidth', 3);
%                         plot(xk1(1), xk1(2), 'go', 'linewidth', 3);
%                         plot(xk(1), xk(2), 'bo', 'linewidth', 3);
%                         dc_dx2 = obj.numericPartial(x);
%                         plot([x(1)-dc_dx(1), x(1), x(1)+dc_dx(1)], [x(2)-dc_dx(2),x(2), x(2)+dc_dx(2)], 'r');
%                         plot([x(1)-dc_dx2(1), x(1), x(1)+dc_dx2(1)], [x(2)-dc_dx2(2),x(2), x(2)+dc_dx2(2)], 'b');
%                     end
                end
                
                % Check for stopping conditions
                if norm(obj.beta^obj.k * dc_dx') < obj.armijo_eps
                    obj.k = 1; % Reset for next armijo iteration 
                    break;
                elseif ~cond1 && ~cond2 % Come to invalid conditions on error in the gradient
                    break;
                end
            end
            
            % Set the lowest cost k_min
            obj.k = k_min;
        end
    end
end

