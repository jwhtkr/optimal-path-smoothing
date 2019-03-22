function launch()
    % Clear previous work
    close all;
    clear;
    clc;
    
    % Configure file path
    addpath Dependencies\intersections
    addpath Scenarios
    addpath Sensors
    addpath VectorFields
    addpath Vehicle
    addpath Vehicle\Kinematics
    addpath Vehicle\LowLevelControl
    addpath World
    
    %%%%%%%%%%%%%%%%% Create scenario %%%%%%%%%%%%%%%%%%%%%
    % Reference and velocity tracking scenarios
    %scenario = ReferenceTrackingScenario(BetterUnicycleVehicle);
    %scenario = VelocityTrackingScenario(BetterUnicycleVehicle);    
    
    % Create a vector field
    x_vec = -5:.5:5;
    y_vec = -5:.5:5;
    field = GoToGoalField(x_vec, y_vec, [3; 4]);
    
    % Vector field scenario
    scenario = VectorFieldScenario(field, BetterUnicycleVehicle, EmptyWorld);
    
    
    
    % Run the scenario
    scenario.runScenario();
end

