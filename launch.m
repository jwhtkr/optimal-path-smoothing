function launch()
    % Clear previous work
    close all;
    clear;
    clc;
    
    % Configure file path
    addpath Dependencies
    addpath Dependencies/intersections
    addpath Scenarios
    addpath Sensors
    addpath VectorFields
    addpath Vehicle
    addpath Vehicle/Kinematics
    addpath World
    addpath TrajectoryPlanners/MultiArcBased/Opt_dep
    addpath TrajectoryPlanners/Clothoid
    addpath TrajectoryPlanners
    
    %%%%%%%%%%%%%%%%% Create scenario %%%%%%%%%%%%%%%%%%%%%
    % Reference tracking scenario
    %scenario = ReferenceTrackingScenario(BetterUnicycleVehicle);
    %scenario = ReferenceTrackingScenario(SimpleUnicycleVehicle);
    %scenario = ReferenceTrackingScenario(SmoothDifferentialDriveVehicle);
    
    % Velocity tracking scenarios
    %scenario = VelocityTrackingScenario(BetterUnicycleVehicle);
    
    %scenario = VelocityTrackingScenario(SimpleUnicycleVehicle);
%     scenario = ParamOptScenario(BetterUnicycleVehicle);
%     scenario = MultiParamOptScenario(BetterUnicycleVehicle);

    
   
% state variation testing sim
%     path = [0 0; 2 4; 5 -2.5; 7.75 4.5];
%     scenario = SingleAgentParamOptScenario(EmptyWorld, BetterUnicycleVehicle([0;0;1.107148717794090;1;0]),path);
    
% Example 1
    path = [0 0; 12 0; 12 8; 20 8; 20 0];
    scenario = SingleAgentParamOptScenario(CorridorWorld, BetterUnicycleVehicle([0;0;0;1;0]),path);
% Example 2
%     path = [0 0; 2 4; 5 -2.5; 7.75 4.5; 11 -2.5; 15 11; 20 8; 20 0];
%     scenario = SingleAgentParamOptScenario(CorridorWorldStraight, BetterUnicycleVehicle([0;0;1.107148717794090;1;0]),path); 

    
    % Create a vector field
    x_vec = -5:.5:5;
    y_vec = -5:.5:5;
    %field = GoToGoalField(x_vec, y_vec, [3; 4], 1);
    %field = AvoidObstacle(x_vec, y_vec, [1; 1], 1);
    %field = OrbitField(x_vec, y_vec, [1; 1], 2, -1, .1);
    %field = LineVectorField(x_vec, y_vec, [0; 0], 0, 1, 1);
    field = LineVectorField(x_vec, y_vec, [-2; 2], pi/4, 1, 1);
    
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
        
    % Run the scenario
    scenario.runScenario();
end


