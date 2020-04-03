classdef DesiredFlatTrajectory < handle
    %DesiredTrajectory is a class that defines the differentially flat
    %trajectory for a unicycle robot
    
    properties(SetAccess=protected)
        dt % Time step size
        t0 % initial time
    end
    
    methods
        function obj = DesiredFlatTrajectory(dt, t0)
            %DesiredFlatTrajectory Construct an instance of this class 
            % Inputs:
            %   dt: time step size
            %   t0: initial time
            obj.dt = dt;
            obj.t0 = t0;
        end
    end
    
    methods(Abstract)
        xd = getDesiredState(obj, t); % Returns the desired state
        ud = getDesiredControl(obj, t); % Returns the desired control
    end
    
    methods
        function [xd, ud] = getDesiredStateAndControl(obj, step)
        %getDesiredStateAndControl returns the desired state and control at 
        %the given discrete time step
            
            % Calculate the time given the time step
            t = obj.getTimeFromStep(step);
            
            % Calculate the 
            xd = obj.getDesiredState(t);
            ud = obj.getDesiredControl(t);
        end
        
        function t = getTimeFromStep(obj, step)
        %getTimeFromStep returns the time corresponding to the discrete
        %step
            t = step*obj.dt + obj.t0;
        end
        
        function xd = getDesiredDiscreteState(obj, step)
        % Returns the desired state given the discrete step
            % Calculate the time given the time step
            t = obj.getTimeFromStep(step);
            
            % Calculate the 
            xd = obj.getDesiredState(t);            
        end
        
        function ud = getDesiredDiscreteControl(obj, step)
        % Returns the desired control given the discrete step
            % Calculate the time given the time step
            t = obj.getTimeFromStep(step);
            
            % Calcualte the desired input
            ud = obj.getDesiredControl(t);
        end
    end
end

