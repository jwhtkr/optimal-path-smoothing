function prob2_3_3()
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
    % Create a vector field
    x_vec = -5:.5:5;
    y_vec = -5:.5:5;
    %field = GoToGoalField(x_vec, y_vec, [3; 4], 1);
    %field = AvoidObstacle(x_vec, y_vec, [1; 1], 1);
    field = OrbitField(x_vec, y_vec, [1; 1], 2, -1, .1);
    %field = LineVectorField(x_vec, y_vec, [-2; 2], pi/4, 1, 1);
    %field = LineVectorField(x_vec, y_vec, [0; 0], 0, 1, 1);
    
    % Vector field scenario
    scenario = VectorFieldScenario(field, SmoothDifferentialDriveVehicle, EmptyWorld, VECTOR_FOLLOWING_TYPE.VELOCITY);
    
    % Run the scenario
    scenario.runScenario();
end

