classdef VectorFieldScenario < Scenario
    %VectorFieldScenario Controls a vehicle to follow a given vector field
    
    properties
        vector_field % Instance of the VectorField class
    end
    
    methods
        function obj = VectorFieldScenario(vector_field, veh, world)
            % Initialize the scenario
            obj = obj@Scenario(veh, world, true);
            
            % Store the vector field
            obj.vector_field = vector_field;            
        end
        
         %%%%  Abstract Method Implementation %%%%
        function u = control(obj, t, x)
            % Get the vector
            g = obj.vector_field.getVector(t, x(obj.q_ind));
            
            % Calculate the velocity control            
            u = obj.vehicle.vectorFieldControl(t, g, x);
        end
        
        function initializeStatePlot(obj)
           initializeStatePlot@Scenario(obj);
           
           % Plot the vector field
           obj.vector_field.plotVectorField(0);           
        end
    end
end

