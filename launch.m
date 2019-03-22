function launch()
    % Clear previous work
    close all;
    clear;
    clc;
    
    % Configure file path
    addpath Dependencies
    addpath Dependencies\intersections
    addpath Scenarios
    addpath Sensors
    addpath VectorFields
    addpath Vehicle
    addpath Vehicle\Kinematics
    addpath World
    
    %%%%%%%%%%%%%%%%% Create scenario %%%%%%%%%%%%%%%%%%%%%
    % Reference tracking scenario
    %scenario = ReferenceTrackingScenario(BetterUnicycleVehicle);
    %scenario = ReferenceTrackingScenario(SimpleUnicycleVehicle);
    
    % Velocity tracking scenarios
    %scenario = VelocityTrackingScenario(BetterUnicycleVehicle);    
    %scenario = VelocityTrackingScenario(SimpleUnicycleVehicle);    
    
    % Create a vector field
    x_vec = -5:.5:5;
    y_vec = -5:.5:5;
    %field = GoToGoalField(x_vec, y_vec, [3; 4], 1);
    %field = OrbitField(x_vec, y_vec, [1; 1], 2, 1, .1);
    %field = LineVectorField(x_vec, y_vec, [-2; 2], pi/4, 1, 1);
    
    % Vector field scenario
    %scenario = VectorFieldScenario(field, BetterUnicycleVehicle, EmptyWorld);
    scenario = VectorFieldScenario(field, SimpleUnicycleVehicle, EmptyWorld);
    
    
    
    % Run the scenario
    scenario.runScenario();
end

