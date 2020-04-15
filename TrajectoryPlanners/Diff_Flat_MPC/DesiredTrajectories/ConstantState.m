classdef ConstantState < DesiredFlatTrajectory
    %ConstantPosition defined the trajectory as a desired position with
    %zero for the derivatives
    
    properties(SetAccess = protected)
        xd % Stores the desired state
        uff % Stores the desired feed forward term
    end
    
    methods
        function obj = ConstantState(dt, t0, xd, uff)
            %ConstandPosition Construct an instance of this class
            %
            % Inputs:
            %   xd: the desired state
            obj = obj@DesiredFlatTrajectory(dt, t0);
            
            % Calculate the desired state
            obj.xd = xd;
            obj.uff = uff;
        end
        
        function xd = getDesiredState(obj, ~)
        % Returns the desired state
            xd = obj.xd;
        end
        
        function ud = getDesiredControl(obj, ~)
        % Returns the desired control
            ud = obj.uff;
        end
    end
end

