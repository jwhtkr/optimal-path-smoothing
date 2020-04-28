function [A, b] = LinearConstraints(points, vectors)
%LINEARCONSTRAINTS Converts lines (hyperplanes) from point, normal to
%matrix representation
%   @param points: An array of points where each column is one point. This
%                  results in an array of size (n,p) where n is the
%                  dimensionality of the point and p is the number of
%                  points (corresponding to number of constraints).
%   @param vectors: An array of direction vectors where each column is one
%                   vector. A column in this array corresponds to a column
%                   in the points input array, giving the same size (n,p).
%                   For the form of linear inequality these should "point"
%                   in the direction of infeasibility. I.e., if the point
%                   is [0;0] and any point to the right of this point is
%                   allowable (right half plane), then the direction vector
%                   would be any [-alpha;0] with alpha positive. Note that
%                   this vector points in the direction of infeasibility
%                   (negative x), as opposed to the direction of
%                   allowability (positive x).
%
%   @return A: The A matrix in the linear inequality A*x <= b. Of size
%              (p,n).
%   @return b: The b vector in the linear inequality A*x <= b. Of size
%              (p,1).
%
%   The conversion between points and direction is as follows for a single
%   point and vector:
%       vector' * x <= vector' * point
%          A    * x <=         b
%   Thus, the A matrix rows are composed of the transposed vectors and the
%   b vector is comprised of the dot product of each vector with its
%   corresponding point.

% Create A and b
A = vectors'; % simple transpose from input vectors is sufficient
b = dot(vectors, points)';  % dot(A,B) computes dot product of each 
                            % A(:,i)*B(:,i) with each output an element in 
                            % a row vector. Transposed to get a column 
                            % vector.
end

