function [A, b] = BoxConstraint(follower_offset, delta)
%BOXCONSTRAINT Create a simple box constraint around the follower 
%   @input x_leader: the position of the leader as an (n,1) vector
%   @input follower_offset: the offset vector from the leader position to
%                           the desire position of the follower as an
%                           (n,1) vector
%   @input delta: a positive offset value for scaling the size of the box.
%                 A side of the box is of length 2*delta.
%
%   @return A: the A matrix of the linear inequality constraint.
%   @return b: the b vector of the linear inequality constraint.

% Extract sizes
[n,~] = size(follower_offset);

% Preallocate
mat = zeros(n);
points = zeros(n,2*n);
vectors = zeros(size(points));

% Get offset direction as a unit vector
follower_dir = follower_offset/norm(follower_offset);

% Get orthogonal directions
mat(:,1) = follower_offset;    % make a matrix to represent the offset vector space
dirs = null(mat');  % find a set of vectors orthogonal to the offset vector
% if follower_offset is a zero vector, then dirs spans R^n, otherwise it
% needs the the follower_dir appended (prepended) to span R^n.
if any(follower_offset ~= zeros(size(follower_offset)))
    dirs = [follower_dir dirs];  % add follower dir
end

% Build remainder of point and vector arrays from remaining directions
for i=1:n
    ind = (i-1)*2 + 1;
    
    dir = dirs(:,i);
    points(:,ind) = follower_offset + delta*dir;
    vectors(:,ind) = dir;
    points(:,ind+1) = follower_offset - delta*dir;
    vectors(:,ind+1) = -dir;
end
% quiver(points(1,:), points(2,:), vectors(1,:), vectors(2,:))
[A, b] = LinearConstraints(points, vectors);
end

