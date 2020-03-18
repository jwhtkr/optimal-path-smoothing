function eulerIntegrateTranslationalRotational(traj, varargin)
%eulerIntegrateDerivatives throws the translational variables into x and
%the rotational variables into y and inputs those trajectories to eulerIntegrateTrajectory

    % Read in the trajectory
    traj_new = Trajectory2D();
    traj_new.copy(traj);
    
    % Store the translational variables in x
    traj_new.x = traj_new.v;
    traj_new.xdot = traj_new.a;
    traj_new.xddot = traj_new.j;
    
    % Store the rotational variables in y
    traj_new.y = traj_new.psi;
    traj_new.ydot = traj_new.w;
    traj_new.yddot = traj_new.alpha;
    
    % Read in the x or y variables
    if nargin > 1
        use_x = varargin{1};
    else
        use_x = true;
    end
    
    % Read in the derivatives variable
    if nargin > 2
        derivatives = varargin{2};
        derivatives = max(1, derivatives); % Need at least 1 derivative
        derivatives = min(2, derivatives); % Cannot do more than 2 derivatives
    else
        derivatives = 2;
    end
    
    % Call eulerIntegrateTrajectory
    eulerIntegrateTrajectory(traj_new, use_x, derivatives);
    

end

