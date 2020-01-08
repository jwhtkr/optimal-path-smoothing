function testFillopovCombination
    for k = 1%:1000000
        % Create the vectors
%         uo = randn(2,1);
        uo = [-1; 0]
        uo1 = uo(1); uo2 = uo(2);
        ut = [1; 1]
%         ut = randn(2,1);

        % find the combination
        a = -(uo'*ut) / (uo'*uo - uo'*ut)

        % Create the combination
        uh = a*uo + (1-a)*ut;

        if a <= 1 && a >= 0
            break;
        end
    end
    
    % Test combination
    err = uh'*uo
    
    %% Plot the results
    figure;
    plot([0 uo1], [0 uo2], 'b'); hold on;
    plot([0 ut(1)], [0 ut(2)], 'r');
    plot([0 uh(1)], [0 uh(2)], 'g');
    legend('uo', 'ut', 'uh')
    axis equal
    
end

