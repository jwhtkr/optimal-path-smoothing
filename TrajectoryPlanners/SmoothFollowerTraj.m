function smoothed_traj = SmoothFollowerTraj(follower_traj, leader_traj, offset, constraints)
%SMOOTHTRAJOPT Uses optimization to smooth an input trajectory
%   @param follower_traj: The follower trajectory to be smoothed as a
%                        Trajectory2D object
%   @param leader_traj: the leader trajectory for reference
%   @param offset: the offset of this follower from the leader
%   @param constraints: the position constraints (from Voronoi) for this follower
xd_mat = zeros(2, 5, length(follower_traj.x));
xd_mat(1,:,:) = [follower_traj.x; follower_traj.xdot; follower_traj.xddot; 
    follower_traj.xdddot; follower_traj.xddddot];
xd_mat(2,:,:) = [follower_traj.y; follower_traj.ydot; follower_traj.yddot; 
    follower_traj.ydddot; follower_traj.yddddot];

end

