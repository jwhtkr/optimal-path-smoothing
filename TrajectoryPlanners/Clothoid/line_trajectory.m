function traj = line_trajectory(p1, p2, v_in, dt)
ds = v_in * dt;
traj = Trajectory2D();
traj.ds = ds;
dx = p2(1) - p1(1);
dy = p2(2) - p1(2);
traj.s_geo = sqrt(dx^2 + dy^2);

% x = atleast_1d(linspace(0, traj.s_geo, max(1, ceil(traj.s_geo / ds)), endpoint=True).squeeze())
x = linspace(0,traj.s_geo,max(1,ceil(traj.s_geo / ds)));
n = length(x);
psi0 = atan2(dy, dx) * ones(1,n);  % heading used to calculate derivatives

% Position variables
traj.x = p1(1) + x .* cos(psi0);
traj.xdot = v_in .* cos(psi0);
traj.xddot = zeros(1,n);
traj.xdddot = zeros(1,n);
traj.xddddot = zeros(1,n);

traj.y = p1(2) + x .* sin(psi0);
traj.ydot = v_in .* sin(psi0);
traj.yddot = zeros(1,n);
traj.ydddot = zeros(1,n);
traj.yddddot = zeros(1,n);

% Orientation variables
traj.psi = ones(1,n) .* psi0;
traj.w = zeros(1,n);
traj.alpha = zeros(1,n);
traj.zeta = zeros(1,n);

% Translational variables
traj.v = ones(1,n) * v_in;
traj.a = zeros(1,n);
traj.j = zeros(1,n);

% Curvature variables
traj.k = zeros(1,n);
traj.sigma = zeros(1,n);

traj.s = 0:ds:n*ds;
traj.s = traj.s(1:n);
traj.t = traj.s ./ traj.v;
traj.cloth_len = n;
traj.dt = dt;

end