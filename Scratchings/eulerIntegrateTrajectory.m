function eulerIntegrateTrajectory(traj, varargin)
    if nargin > 1
        use_x = varargin{1};
    else
        use_x = true;
    end
    
    if nargin > 2
        derivatives = varargin{2};
        derivatives = max(1, derivatives); % Need at least 1 derivative
        derivatives = min(4, derivatives); % Cannot do more than 4 derivatives
    else
        derivatives = 4;
    end
    
    if use_x
        x0 = [traj.x(1); traj.xdot(1); traj.xddot(1); traj.xdddot(1)];
        
        x_ = traj.x;
        xdot_ = traj.xdot;
        xddot_ = traj.xddot;
        xdddot_ = traj.xdddot;
        xddddot_ = traj.xddddot;
        
        ylabels = {'x', 'xdot', 'xddot', 'xdddot', 'xddddot'};
    else
        x0 = [traj.y(1); traj.ydot(1); traj.yddot(1); traj.ydddot(1)];
        
        x_ = traj.y;
        xdot_ = traj.ydot;
        xddot_ = traj.yddot;
        xdddot_ = traj.ydddot;
        xddddot_ = traj.yddddot;
        
        ylabels = {'y', 'ydot', 'yddot', 'ydddot', 'yddddot'};
    end
    x_vals = {x_, xdot_, xddot_, xdddot_, xddddot_};
    u = x_vals{derivatives+1};

    % Setup full dynamics
    A = [0 1 0 0; 0 0 1 0; 0 0 0 1; 0 0 0 0];
    B = [0; 0; 0; 1];
    
    % Downsample based on derivatives
    n_start = 4 - derivatives + 1;
    A = A(n_start:end, n_start:end);
    B = B(n_start:end);
    x0 = x0(1:derivatives);
    
    % Integrate forward
    x = x0;
    x_int = [x];    
    for k = 1:(length(u)-1)
        x = x + traj.dt*(A*x + B*u(k));
        x_int = [x_int x];
    end

    % Plot the states
    figure;
    for k = 1:derivatives+1        
        subplot(derivatives+1,1,k);
        plot(x_vals{k}, 'b'); hold on;
        if k < derivatives + 1
            plot(x_int(k,:), 'r');
        end
        ylabel(ylabels{k})
    end

%     subplot(5,1,2);
%     plot(xdot_, 'b'); hold on;
%     plot(x_int(2,:), 'r');
%     ylabel(ylabels{2})
% 
%     subplot(5,1,3);
%     plot(xddot_, 'b'); hold on;
%     plot(x_int(3,:), 'r');
%     ylabel(ylabels{3})
% 
%     subplot(5,1,4);
%     plot(xdddot_, 'b'); hold on;
%     plot(x_int(4,:), 'r');
%     ylabel(ylabels{4})
% 
%     subplot(5,1,5);
%     plot(xddddot_, 'b'); hold on;
%     plot(xddddot_, 'r');
%     ylabel(ylabels{5})
end