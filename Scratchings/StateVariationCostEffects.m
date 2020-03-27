
% Clear previous work
clc; close all; clear all;

% Sim variables
dt = .01;       % sim time step
zdot = [];      % zdot = xdot - xdot_desired
L = [];         % instantaneous cost
dphi_dx = [];   % partial of the terminal cost wrt state    
cost = [];      % cost of contorl trajectory over mpc horizon
u_agent = [0;0;0;0;0;0;0;0;0];  % zero control, defaults to trajectory tracking controller


% Trajectory data
path = [0 0; 20 0; 20 20]; % Trivial Trajectory
trajectory = TrajUtil.createClothoidTrajectory(path, 1, dt, .85, .85);

% Vehicle Variables
vehicle = BetterUnicycleVehicle([0;0;0;0;0]);
vehicle.u_init = u_agent;

% MPC Variables
mpc = Unicycle2(vehicle.x);
mpc.trajectory = trajectory; % Set the trajectory of the MPC object
tf = (size(trajectory.x,2)-1)*dt - mpc.tf - dt; % Set the final time of the sim
mpc.xd = trajectory.stateAtTime(mpc.tf); % Set the desired state at the end of mpc horizon







n = 21;
x_pos_range = linspace(-10,10,n);
y_pos_range = linspace(-10,10,n);
yaw_range = linspace(-pi,pi,n);
vel_range = linspace(-2,2,n);
yaw_rate_range = linspace(-1,1,n);
cond4_inequality = zeros(n);

t = 0;
x = [0; 0; 0; 1; 0];
ind = 1;
for i = 1:length(x_pos_range)
    x(ind) = x_pos_range(i);
    dphi_dx = mpc.terminalStatePartial(x);
    zdot = mpc.unicycleDualModeDynamics(mpc.dt, x, u_agent) - trajectory.xdotAtTime(t);
    L = mpc.instantaneousCost(mpc.dt, x, u_agent);
    cond4_inequality(ind,i) = dphi_dx*zdot + L;
%     cost = [cost; mpc.cost(u_agent)];
end

x = [0; 0; 0; 1; 0];
ind = 2;
for i = 1:length(y_pos_range)
    x(ind) = y_pos_range(i);
    dphi_dx = mpc.terminalStatePartial(x);
    zdot = mpc.unicycleDualModeDynamics(mpc.dt, x, u_agent) - trajectory.xdotAtTime(t);
    L = mpc.instantaneousCost(mpc.dt, x, u_agent);
    cond4_inequality(ind,i) = dphi_dx*zdot + L;
%     cost = [cost; mpc.cost(u_agent)];
end

x = [0; 0; 0; 1; 0];
ind = 3;
for i = 1:length(yaw_range)
    x(ind) = yaw_range(i);
    dphi_dx = mpc.terminalStatePartial(x);
    zdot = mpc.unicycleDualModeDynamics(mpc.dt, x, u_agent) - trajectory.xdotAtTime(t);
    L = mpc.instantaneousCost(mpc.dt, x, u_agent);
    cond4_inequality(ind,i) = dphi_dx*zdot + L;
%     cost = [cost; mpc.cost(u_agent)];
end

x = [0; 0; 0; 1; 0];
ind = 4;
for i = 1:length(vel_range)
    x(ind) = vel_range(i);
    dphi_dx = mpc.terminalStatePartial(x);
    zdot = mpc.unicycleDualModeDynamics(mpc.dt, x, u_agent) - trajectory.xdotAtTime(t);
    L = mpc.instantaneousCost(mpc.dt, x, u_agent);
    cond4_inequality(ind,i) = dphi_dx*zdot + L;
%     cost = [cost; mpc.cost(u_agent)];
end

x = [0; 0; 0; 1; 0];
ind = 5;
for i = 1:length(yaw_rate_range)
    x(ind) = yaw_rate_range(i);
    dphi_dx = mpc.terminalStatePartial(x);
    zdot = mpc.unicycleDualModeDynamics(mpc.dt, x, u_agent) - trajectory.xdotAtTime(t);
    L = mpc.instantaneousCost(mpc.dt, x, u_agent);
    cond4_inequality(ind,i) = dphi_dx*zdot + L;
%     cost = [cost; mpc.cost(u_agent)];
end

figure;
ind = 1;
subplot(5,1,ind)
plot(x_pos_range,cond4_inequality(ind,:)); hold on;
plot([x_pos_range(1) x_pos_range(end)],[0 0], ':r');
title('State Variation Effect on Condition 4')
xlabel('x position')

ind = 2;
subplot(5,1,ind)
plot(y_pos_range,cond4_inequality(ind,:)); hold on;
plot([y_pos_range(1) y_pos_range(end)],[0 0], ':r');
xlabel('y position')

ind = 3;
subplot(5,1,ind)
plot(yaw_range,cond4_inequality(ind,:)); hold on;
plot([yaw_range(1) yaw_range(end)],[0 0], ':r');
xlabel('yaw')

ind = 4;
subplot(5,1,ind)
plot(trajectory.v(1) - vel_range ,cond4_inequality(ind,:)); hold on;
plot(trajectory.v(1) - [vel_range(1) vel_range(end)],[0 0], ':r');
xlabel('velocity')

ind = 5;
subplot(5,1,ind)
plot(yaw_rate_range,cond4_inequality(ind,:)); hold on;
plot([yaw_rate_range(1) yaw_rate_range(end)],[0 0], ':r');
xlabel('yaw rate')

