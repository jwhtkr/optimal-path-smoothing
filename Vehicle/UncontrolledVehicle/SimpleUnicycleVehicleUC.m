classdef SimpleUnicycleVehicleUC < UncontrolledVehicle
    methods
        function obj = SimpleUnicycleVehicleUC(varargin)
           % Get the initial state
            x0 = [0 0 0]'; % default to the zero state
            if nargin > 0
                x0 = varargin{1}; 
            end
            
            % Initialize the kinematics and the vehicle
            kin = SimpleUnicycle;
            q_ind = [kin.x_ind; kin.y_ind];
            obj = obj@UncontrolledVehicle(kin, x0, q_ind);
        end
    end
end

