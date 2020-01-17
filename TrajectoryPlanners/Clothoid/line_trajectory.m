function traj = line_trajectory(p1, p2, v, dt)
ds = v * dt;
traj = Trajectory2D();
traj.ds = ds;
dx = p2(1) - p1(1);
dy = p2(2) - p1(2);
traj.s_geo = sqrt(dx^2 + dy^2);

% x = atleast_1d(linspace(0, traj.s_geo, max(1, ceil(traj.s_geo / ds)), endpoint=True).squeeze())
x = linspace(0,traj.s_geo,max(1,ceil(traj.s_geo / ds)));
n = length(x);
psi = atan2(dy, dx) * ones(1,n);  % heading used to calculate derivatives

traj.x = p1(1) + x .* cos(psi);
traj.xdot = v .* cos(psi);
traj.xddot = zeros(1,n);
traj.xdddot = zeros(1,n);

traj.y = p1(2) + x .* sin(psi);
traj.ydot = v .* sin(psi);
traj.yddot = zeros(1,n);
traj.ydddot = zeros(1,n);

traj.psi = ones(1,n) .* psi;
traj.v = ones(1,n) * ds / dt;
traj.a = zeros(1,n);
traj.sigma = zeros(1,n);
traj.a = zeros(1,n);
traj.alpha = zeros(1,n);
traj.k = zeros(1,n);
traj.sigma = zeros(1,n);
traj.s = 0:ds:n*ds;
traj.s = traj.s(1:n);
traj.t = traj.s ./ traj.v;

end