function smoothed_traj = SmoothTraj2D(desired_traj, Q, R, S)
%SMOOTHTRAJOPT Uses optimization to smooth an input trajectory
%   @param desired_traj: The trajectory to be smoothed as a
%                        Trajectory2D object.
%   @param Q: the instantaneous weights on the state errors
%   @param R: the instantaneous weights on the input (snap)
%   @param S: the terminal weights on the state errors
xd_mat = zeros(2, 5, length(desired_traj.x));
xd_mat(1,:,:) = [desired_traj.x; desired_traj.xdot; desired_traj.xddot; desired_traj.xdddot; desired_traj.xddddot]';
xd_mat(2,:,:) = [desired_traj.y; desired_traj.ydot; desired_traj.yddot; desired_traj.ydddot; desired_traj.yddddot]';
% xd = reshape(xd_mat, [], 1);
end

