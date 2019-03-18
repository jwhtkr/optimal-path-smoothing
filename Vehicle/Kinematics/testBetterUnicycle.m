function testBetterUnicycle()
    
    % Create the vehicle
    veh = BetterUnicycle;
    
    % Select the control
    u = @(t,x)constantRadiusUnicycle(t,x);
    %u = @(t,x)constantRadiusBicycle(t,x);
    
    % Select the integration mode
    integrator = @(t0, dt, tf, x0, veh, u) integrateODE(t0, dt, tf, x0, veh, u);
    %integrator = @(t0, dt, tf, x0, veh, u) integratorEuler(t0, dt, tf, x0, veh, u);
    
    % Integrate the state
    t0 = 0; 
    dt = 0.1;
    tf = 10;
    x0 = [0; 0; 0; 0; 0];
    [tmat, xmat] = integrator(t0, dt, tf, x0, veh, u);
    
    % Plot the state
    for k = 1:length(tmat)
       veh.plotState(tmat(k), xmat(:,k));
       pause(dt);
    end
    
end

function [tmat, xmat] = integrateODE(t0, dt, tf, x0, veh, u)
% Input parameters
%   t0: initial time
%   dt: time step for return data
%   tf: final time
%   x0: initial state
%   veh: instantiation of VehicleKinematics class
%   u: function handle which takes input arguments of (t,x)

    % Integrate forward in time
    [tmat, xmat] = ode45(@(t,x)veh.kinematics(t, x, u(t,x)), [t0:dt:tf], x0);
    
    % Transpose the outputs
    tmat = tmat';
    xmat = xmat';    
end

function [tmat, xmat] = integratorEuler(t0, dt, tf, x0, veh, u)
% Input parameters
%   t0: initial time
%   dt: time step for return data
%   tf: final time
%   x0: initial state
%   veh: instantiation of VehicleKinematics class
%   u: function handle which takes input arguments of (t,x)

    % Initialize state data
    tmat = [t0:dt:tf]';
    len = length(tmat);
    xmat = zeros(veh.dimensions, len);
    xmat(:,1) = x0;
    
    % Loop through and calculate the state
    x = x0;
    for k = 1:len
        % Calculate state update equation
        t = tmat(k);
        xdot = veh.kinematics(t, x, u(x,t));
        
        % Update the state
        x = x + dt * xdot;
        
        % Store the state
        xmat(:,k) = x;
    end

end

function u = constantRadiusUnicycle(t, x)
    u = [1; 1];
end

function u = constantRadiusBicycle(t, x)
    u = [1; 0.7854]; % .7854 corresponds to a steering angle that will produce w = 1 for L and v = 1
end

