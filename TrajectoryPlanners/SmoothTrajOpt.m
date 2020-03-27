function smoothed_traj = SmoothTrajOpt(traj_mat, Q, R, S, A_c, b_c)
%SMOOTHTRAJOPT Uses optimization to smooth an input trajectory
%   @param traj_mat: the input trajectory as a matrix of size: (n,m,N)
%                    where n is the dimensionality (2D, 3D, etc.), m is the 
%                    number of derivatives (determining the class of 
%                    smoothnes), and N is the number of time steps in the 
%                    trajectory.
%   @param Q: the instantaneous cost on nearness to input trajectory values
%   @param R: the instantaneous cost on input values
%   @param S: the terminal cost on nearness to last input trajectory value
%   @param A_c: the linear inequality constraints for each time step as a
%               matrix of size: (p,n,m,N) with n,m,N as before and p the
%               number of constraints at each time instance.
%   @param b_c: the linear inequality constraints for each time step as a
%               matrix of size: (p,N) with p,N as previously defined.


end