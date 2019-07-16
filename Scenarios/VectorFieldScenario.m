classdef VectorFieldScenario < Scenario
    %VectorFieldScenario Controls a vehicle to follow a given vector field
    
    properties
        vector_field % Instance of the VectorField class
        control_type % The type of vector field control to be used - instance of the VECTOR_FOLLOWING_TYPE
    end
    
    methods
        function obj = VectorFieldScenario(vector_field, veh, world, control_type)
            % Initialize the scenario
            obj = obj@Scenario(veh, world, true);
            obj.tf = 40; % Simulate for 20 seconds
            
            % Store the vector field
            obj.vector_field = vector_field; 
            obj.control_type = control_type;
        end
        
         %%%%  Abstract Method Implementation %%%%
        function u = control(obj, t, x)
            % Get the vector
            g = @(t_val, x_vec, th)obj.vector_field.getVector(t_val, x_vec, th);
            
            % Calculate the velocity control            
            u = obj.vehicle.vectorFieldControl(t, g, obj.control_type, x);
        end
        
        function initializeStatePlot(obj)
           initializeStatePlot@Scenario(obj);
           
           % Plot the vector field
           obj.vector_field.plotVectorField(0);           
        end
    end
end

