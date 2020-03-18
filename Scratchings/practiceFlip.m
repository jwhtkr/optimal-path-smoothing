function practiceFlip
close all;
    dt = 0.001;
    t0 = 0;
    tf = 10;
    t = t0:dt:tf;
    
    x = cos(t);
    xdot = -sin(t);
    xddot = -cos(t);
    xdddot = sin(t);
    xddddot = cos(t);
    
    evaluate();
    
    %% Flip the state
    x = -flip(x);
    xdot = flip(xdot);
    xddot = -flip(xddot);
    xdddot = flip(xdddot);
    xddddot = -flip(xddddot);
    
    evaluate();
    
    
    function evaluate()
        % Form the aggregate state
        x_state = [x; xdot; xddot; xdddot; xddddot];    

        % Integrate forward in time
        x0 = x_state(1:4, 1);
        x_int = eulerIntegrate(x0, xddddot, dt, t0, tf);

        % Plot the states
        figure;
        subplot(5,1,1);
        plot(x, 'b'); hold on;
        plot(x_int(1,:), 'r');
        ylabel('x')

        subplot(5,1,2);
        plot(xdot, 'b'); hold on;
        plot(x_int(2,:), 'r');
        ylabel('xdot')

        subplot(5,1,3);
        plot(xddot, 'b'); hold on;
        plot(x_int(3,:), 'r');
        ylabel('xddot')

        subplot(5,1,4);
        plot(xdddot, 'b'); hold on;
        plot(x_int(4,:), 'r');
        ylabel('xdddot')
        
        subplot(5,1,5);
        plot(xddddot, 'b'); hold on;
        plot(xddddot, 'r');
        ylabel('xddddot')
    end
    
    
end

function xmat = eulerIntegrate(x0, u, dt, t0, tf)
    xmat = [x0];
    x = x0;
    
    A = [0 1 0 0; 0 0 1 0; 0 0 0 1; 0 0 0 0];
    B = [0; 0; 0; 1];
    for k = 1:(length(u)-1)
        x = x + dt*(A*x + B*u(k));
        xmat = [xmat x];
    end
end

