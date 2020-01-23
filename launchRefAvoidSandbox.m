function launchRefAvoidSandbox()
    % Clear previous work
    close all;
    clear;
    clc;
    
    % Configure file path
    addpath Agents
    addpath Dependencies
    addpath Dependencies\intersections
    addpath Plotters
    addpath Scenarios
    addpath Scenarios\MultiAgentScenarios
    addpath Sensors
    addpath TrajectoryPlanners\Clothoid
    addpath VectorFields
    addpath Vehicle
    addpath Vehicle\Kinematics
    addpath World
    
    %%%%%%%%%%%%%%%%% Create scenario (old way) %%%%%%%%%%%%%%%%%%%%%
    
%     % Reference avoid scenario
%     %waypoints = [0.2 0; 7 0; 13 1; 15 0; 20 1; 25 -6; 16 -6];
%     waypoints = [0.2 1; 10 -4; 15 0; 16 6; 8 8];
%     %scenario = ReferenceAvoidScenario(SmoothDifferentialDriveVehicle, PolygonWorld1, waypoints );    
%     scenario = ReferenceAvoidScenario(SimpleUnicycleVehicle, PolygonWorld1, waypoints );
%     scenario.tf = 30;
%     
%     % Run the scenario
%     scenario.runScenario();
    
    %%%%%%%%%%%%%%%%% Create scenario (new way) %%%%%%%%%%%%%%%%%%%%%
    
    % Reference avoid scenario
    waypoints{1} = [0.2 0; 7 0; 13 1; 15 0; 20 1; 25 -6; 16 -6];
    waypoints{2} = [0.2 1; 10 -4; 15 0; 16 6; 8 8];
    waypoints{3} = [0.2 2; 0.2 8; 20 1; 20 -8; 0 -8];
    
    x0{1} = [0;0;0];
    x0{2} = [-1; 0; 0];
    x0{3} = [1; 0; pi/2];
    %scenario = MultiReferenceAvoidScenario(@SmoothDifferentialDriveVehicle, PolygonWorld1, waypoints, x0);
    scenario = MultiReferenceAvoidScenario(@SimpleUnicycleVehicle, PolygonWorld1, waypoints, x0);
    scenario.tf = 30;
    
    % Run the scenario
    scenario.runScenario();
end

