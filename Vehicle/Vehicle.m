classdef Vehicle < handle
    %Vehicle is a simple abstract interface to enable construction of
    %different vehicles with different sensor configurations.
    %   A vehicle consists of the following elements:
    %       * Kinematic model
    %       * Controllers
    %           * Vector controller
    %           * Path controller
    %       * Sensor - detects nearby obstacles
        
    properties
        % Elements of the vehicle
        kinematics
        sensor
        
        % Vehicle state
        t =0    % Latest time value
        x       % State of the vehicle
        q_ind   % Indices of the position
        th_ind  % Index of the orientation
        x_ind;
        y_ind;
        
        % Latest sensor measurements
        xo_latest = [] % Latest x measurement
        yo_latest = [] % Latest y measurement
        dist_latest = [] % Latest distance
    end
    
    methods (Abstract)
        u = velocityControl(obj, vd, wd, varargin);
        u = pathControl(obj, t, q_des, qd_des, qdd_des, varargin); 
        u = vectorFieldControl(obj, t, g, control_type, varargin);
    end
    
    methods
        function obj = Vehicle(kinematics, x0, q_ind)
            % Store elements
            obj.kinematics = kinematics;
            obj.sensor = RangeSensor;
            obj.x = x0;
            obj.q_ind = q_ind;
            obj.th_ind = kinematics.th_ind;
            obj.x_ind = kinematics.x_ind;
            obj.y_ind = kinematics.y_ind;
            
        end
        
        function [xo, yo, dist] = getObstacleDetections(obj, world)
            % Get the current position and orientation
            q = obj.x(obj.q_ind);
            th = obj.x(obj.th_ind);
            
            % Calculate the distance
            [xo, yo, dist] = obj.sensor.getObstacleDetections(q, th, world);
            
            % Store the data
            obj.xo_latest = xo;
            obj.yo_latest = yo;
            obj.dist_latest = dist;
        end
        
        function initializePlots(obj, ax)
            obj.sensor.initializePlots(ax);
            obj.kinematics.initializeStatePlot(ax, obj.x);
        end
        
        function plotVehicle(obj)
            % Plot the sensor data (if any sensor data exists)
%             if ~isempty(obj.xo_latest)
%                 q = obj.x(obj.q_ind);
%                 obj.sensor.plotMeasurements(q, obj.xo_latest, obj.yo_latest);
%             end
            
            % Plot the vehicle
            obj.kinematics.plotState(obj.t, obj.x);
            
            % Plot the sensor data
            obj.sensor.plotMeasurements(obj.x(obj.q_ind), obj.xo_latest, obj.yo_latest);
        end
        
        function x_config = getConfiguration(obj, t, varargin)
            %getConfiguration returns the configuration of the vehicle
            %(x,y,theta)
            %
            % Inputs:
            %   t: time index of concern (not used by default)
            %   varargin{1}: state (uses vehicle stored state if this is
            %   not input
            %
            % Ouput:
            %   x_config: Configuration of the vehicle in the configuration
            %   space (position, orientation)
            
            % Get the state
            if nargin > 2
                state = varargin{1};
            else
                state = obj.x;
            end
            
            % Return the configuration
            x_config = [state(obj.q_ind); state(obj.th_ind)];
        end
    end
end

