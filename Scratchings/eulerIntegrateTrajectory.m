function eulerIntegrateTrajectory(traj)
            
    x0 = [traj.x(1); traj.xdot(1); traj.xddot(1); traj.xdddot(1)];
    u = traj.xddddot;

    x_int = [x0];
    x = x0;


    A = [0 1 0 0; 0 0 1 0; 0 0 0 1; 0 0 0 0];
    B = [0; 0; 0; 1];
    for k = 1:(length(u)-1)
        x = x + traj.dt*(A*x + B*u(k));
        x_int = [x_int x];
    end

    % Plot the states
    figure;
    subplot(5,1,1);
    plot(traj.x, 'b'); hold on;
    plot(x_int(1,:), 'r');
    ylabel('x')

    subplot(5,1,2);
    plot(traj.xdot, 'b'); hold on;
    plot(x_int(2,:), 'r');
    ylabel('xdot')

    subplot(5,1,3);
    plot(traj.xddot, 'b'); hold on;
    plot(x_int(3,:), 'r');
    ylabel('xddot')

    subplot(5,1,4);
    plot(traj.xdddot, 'b'); hold on;
    plot(x_int(4,:), 'r');
    ylabel('xdddot')

    subplot(5,1,5);
    plot(traj.xddddot, 'b'); hold on;
    plot(traj.xddddot, 'r');
    ylabel('xddddot')



end