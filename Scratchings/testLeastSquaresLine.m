function testLeastSquaresLine()
%Tests a least square fit to a line
%
% Results: runs about 100 times faster than Matlab's polyfit and also deals
% with vertical lines
    % Generate some points
    b = rand*5-2.5;
    m = rand*5-2.5;
    q = pointsFromLine(b, m, 10, true);
    
%     q = [10, 9, 5, 2, 1; 1, 1, 1, 1, 1]; % horizontal line test
%      q = [ 1, 1, 1, 1, 1; 10, 9, 5, 2, 1]; % vertical line test

    q = [ [0;0] [1; 0] [2; 0] [ 5; 0] [5; 4] [5; 5]]; % Corner test
    
    % Calculate a point and vector
    for i = 1:1
%         n = 10000;
%         tic
%         for k = 1:n
            [q0, v] = leastSquaresLine(q);
%         end
%         time_exec = toc
        
%         x = q(1,:)';
%         y = q(2,:)';
%         tic
%         for k = 1:n
%             p = polyfit(x, y, 1);
%         end
%         time_exec_polyfit = toc
    end
    q1 = 5*v+q0;
    
    % Plot the results
    figure;
    plot(q(1,:), q(2,:), 'ro'); hold on;
    plot([q0(1) q1(1)], [q0(2) q1(2)], 'b');
    
end

function q = pointsFromLine(b, m, n, noise)
    q = zeros(2, n);
    for k = 1:n
        x = rand*10-5;
        q(:,k) = [x; m*x+b + noise*.1*randn];
    end
end