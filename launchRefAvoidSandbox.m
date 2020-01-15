function launchRefAvoidSandbox()
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
    %scenario = ReferenceTrackingScenario(SmoothDifferentialDriveVehicle);
    
    % Velocity tracking scenarios
    %scenario = VelocityTrackingScenario(BetterUnicycleVehicle);    
    %scenario = VelocityTrackingScenario(SimpleUnicycleVehicle);    
    
    % Create a vector field
    x_vec = -5:.5:5;
    y_vec = -5:.5:5;
    %field = GoToGoalField(x_vec, y_vec, [3; 4], 1);
    %field = AvoidObstacle(x_vec, y_vec, [1; 1], 1);
    %field = OrbitField(x_vec, y_vec, [1; 1], 2, -1, .1);
    %field = LineVectorField(x_vec, y_vec, [0; 0], 0, 1, 1);
    %field = LineVectorField(x_vec, y_vec, [-2; 2], pi/4, 1, 1);    
    
    % Vector field scenario
    %scenario = VectorFieldScenario(field, BetterUnicycleVehicle, EmptyWorld, VECTOR_FOLLOWING_TYPE.POINT );
    %scenario = VectorFieldScenario(field, SimpleUnicycleVehicle, EmptyWorld, VECTOR_FOLLOWING_TYPE.POINT );
    %scenario = VectorFieldScenario(field, SmoothDifferentialDriveVehicle, EmptyWorld, VECTOR_FOLLOWING_TYPE.POINT);
    
    % Summed vector fields
    %scenario = CombinedGoToGoalVectorScenario(BetterUnicycleVehicle, VECTOR_FOLLOWING_TYPE.POINT );
    %scenario = CombinedGoToGoalOrbitAvoidScenario(BetterUnicycleVehicle, VECTOR_FOLLOWING_TYPE.POINT );
    %scenario = CombinedGoToGoalOrbitAvoidWithBarrierScenario(BetterUnicycleVehicle, VECTOR_FOLLOWING_TYPE.POINT );
    
    
    % Switching vector fields
    %scenario = SwithingLineScenario(BetterUnicycleVehicle, VECTOR_FOLLOWING_TYPE.POINT );
    %scenario = SwithingLineScenarioObstacleAvoid(BetterUnicycleVehicle, VECTOR_FOLLOWING_TYPE.POINT );
    %scenario = SwithingLineScenarioObstacleAvoidBetterSwitch(BetterUnicycleVehicle, VECTOR_FOLLOWING_TYPE.POINT );       
    
    % Reference avoid scenario
    %scenario = ReferenceAvoidScenario(SmoothDifferentialDriveVehicle, PolygonWorld1 );
    scenario = ReferenceAvoidScenario(SimpleUnicycleVehicle, PolygonWorld1 );
    scenario.tf = 30;
    
    % Run the scenario
    scenario.runScenario();
end

