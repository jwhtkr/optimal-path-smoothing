classdef UncontrolledVehicle < handle
    %UncontrolledVehicle is a simple abstract interface to enable construction of
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
        controller % Instance of the Controller object
        sensor
        
        % Vehicle state
        t =0    % Latest time value
        x       % State of the vehicle
        q_ind   % Indices of the position
        th_ind  % Index of the orientation
        x_ind;  % Index of the x position
        y_ind;  % Index of the y position
        
        % Latest sensor measurements
        xo_latest = [] % Latest x measurement
        yo_latest = [] % Latest y measurement
        dist_latest = [] % Latest distance
    end
    
    methods
        function obj = Vehicle(kinematics, x0, q_ind)
            % Store elements
            obj.kinematics = kinematics;
            obj.controller = ZeroControl(obj, obj.kinematics.n_u);
            obj.sensor = RangeSensor;
            obj.x = x0;
            obj.q_ind = q_ind;
            obj.th_ind = kinematics.th_ind;
            obj.x_ind = kinematics.x_ind;
            obj.y_ind = kinematics.y_ind;
        end
        
        function setController(obj, controller)
            obj.controller = controller;            
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
%             obj.sensor.initializePlots(ax);
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
%             obj.sensor.plotMeasurements(obj.x(obj.q_ind), obj.xo_latest, obj.yo_latest);
        end        
        
        function u = calculateControl(obj, t, x)
            u = obj.controller.calculateControl(obj, t, x);
        end
    end
end

