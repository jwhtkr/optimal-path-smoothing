classdef ConstantPosition < DesiredFlatTrajectory
    %ConstantPosition defined the trajectory as a desired position with
    %zero for the derivatives
    
    properties(SetAccess = protected)
        xd % Stores the desired state
    end
    
    methods
        function obj = ConstantPosition(dt, t0, qd,n_derivatives)
            %ConstandPosition Construct an instance of this class
            %
            % Inputs:
            %   qd: the desired position
            %   n_derivatives: the number of derivatives
            obj = obj@DesiredFlatTrajectory(dt, t0);
            
            % Calculate the desired state
            assert(size(qd, 1) == 2 && size(qd, 2) == 1, 'desired position incorrect size');
            obj.xd = [qd; zeros(2*n_derivatives, 1)];
        end
        
        function xd = getDesiredState(obj, ~)
        % Returns the desired state
            xd = obj.xd;
        end
        
        function ud = getDesiredControl(~, ~)
        % Returns the desired control
            ud = [0;0];
        end
    end
end

