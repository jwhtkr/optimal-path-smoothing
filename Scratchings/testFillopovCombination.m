function testFillopovCombination
    for k = 1%:1000000
        % Create the vectors
%         u_o = randn(2,1);
%         uo = [-1; 0]
        u_o = [0.33736; 0.43373];
        
%         u_t = [1; 1]
%         ut = randn(2,1);
        u_t = [1.567; 0.98561];

        % find the combination
        a = -(u_o'*u_t) / (u_o'*u_o - u_o'*u_t)

        % Create the combination
        u_h = a*u_o + (1-a)*u_t

        if a <= 1 && a >= 0
            break;
        end
    end
    
    % Test combination
    err = u_h'*u_o
    nrm_uo = norm(u_o)
    nrm_ut = norm(u_t)
    nrm_uh = norm(u_h)
    
    % check conditions
    u_d = u_t; % ud is the direction that we will travel
    if a >= 0  && a <= 1 % Need to use some combination
        % Compute a normal uh (just unit vector in direction of uo)
        n = u_o ./ norm(u_o);
        
        % Compute needed dot products for decision
        n_dot_uo = n'*u_o;
        n_dot_ut = n'*u_t;
        
        if n_dot_uo*n_dot_ut > 0 % Go in direction of a vector field
            if n_dot_uo < 0
                disp('Move with obstacle avoid');
                u_d = u_o;
            else
                disp('Move with tracking');
                u_d = u_t;
            end
        else % Sliding mode
            disp('Sliding mode');
            u_d = u_h;
        end
    else
        disp('Keep tracking');
        u_d = u_t;
    end
    
    % Scale ud for plotting
    len = min([nrm_uo, nrm_ut, nrm_uh]) ./ 2;
    ud_plot = u_d .* (len/norm(u_d));
    
    
    %% Plot the results
    figure;
    plot([0 u_o(1)], [0 u_o(2)], 'r'); hold on;
    plot([0 u_t(1)], [0 u_t(2)], 'g');
    plot([0 u_h(1)], [0 u_h(2)], 'b');
    %plot([0 ud_plot(1)], [0 ud_plot(2)], 'k');
    plot([0 u_d(1)], [0 u_d(2)], 'k');
    plot(0, 0, 'o', 'linewidth', 2);
    legend('uo', 'ut', 'uh', 'ud', 'zero')
    axis equal
    
    if true
        return;
    end
    
    figure;
    plot([0 u_o(1)], [0 u_o(2)], 'r'); hold on;
    plot([0 u_t(1)], [0 u_t(2)], 'g');
    plot([0 u_h(1)], [0 u_h(2)], 'b');
    plot(0, 0, 'o', 'linewidth', 2);
    legend('uo', 'ut', 'uh', 'zero')
    axis equal
end

