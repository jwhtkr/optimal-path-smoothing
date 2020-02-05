function [q0, v, u] = leastSquaresLine(q_in)
%leastSquaresLine uses linear regression to calculate a straight line
%using the (x,y) pairs in q_in. The line returned is a point and a
%direction such that all points lie in the set
% {q| \exists a \in \R s.t. q = q0 + av}
% 
%
%The approach taken is to use a first order polynomial fit and detect when
%the points are vertical. 
%
%There are two special cases (both will return a vertical line):
% 1) A single point, in this case any line passing through the point is a
% valid least squares fit. The line returned will be a vertical line
% 2) All points are the same. In this case a vertical line will be returned
% as it corresponds to the line with all x-values being the same
%
% Runs about 100 times faster than Matlab's polyfit and also deals
% with vertical lines
%
% Inputs:
%   q_in: 2xm matrix of points where the first row contains the x values
%   and the second row contains the y values
%   
% Outputs:
%   q0: 2x1 Point on the line
%   v: 2x1 Vector of the line starting at initial point and moving in
%      direction of line
%   u: polynomial terms of the line (y = u(1) + u(2)*x) (returns nans if
%      line is vertical)

    % Extract data needed for the least squares fit
    x = q_in(1,:)'; % The input x data
    g = q_in(2,:)'; % The input y data
    m = length(x); % Number of data points
    fat_one = ones(1, m); % Row vector of ones
    
    % Formulate the quadradic term of the least squares
    diag = fat_one*x;
    Q = [m diag; diag x'*x];
    
    % Check the invertibility of the quadratic term
    determinant = Q(1,1)*Q(2,2) - diag^2;
    if (determinant) < .00001 % special case: vertical line
        q0 = q_in(:,1);
        v = [0; 1];
        u = [nan; nan];
        return;
    end
    
    % Calculate the linear term
    b = [fat_one*g; x'*g];
    
    % Calculate the least squares polynomial fit
    Q_inv = 1/determinant .* [Q(2,2), -Q(1,2); -Q(2,1) Q(1,1)];
    u = Q_inv'*b;
    
    % Extract a point
    q0 = [x(1); u(2)*x(1) + u(1)];
    
    % Extract a unit vector
    v = [1; u(2)]./sqrt(1+u(2)^2);    
end