function launchRefAvoidSandbox()
    % Clear previous work
    close all;
    clear;
    clc;
    
    % Configure file path
    addpath Agents
    addpath ComplexBehaviors/WallFollowing
    addpath Dependencies
    addpath Dependencies/distinguishable_colors
    addpath Dependencies/intersections
    addpath Map
    addpath Plotters
    addpath Scenarios
    addpath Scenarios/MultiAgentScenarios
    addpath Sensors
    addpath TrajectoryPlanners
    addpath TrajectoryPlanners/Clothoid
    addpath VectorFields
    addpath Vehicle
    addpath Vehicle/Kinematics
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
    
%     % Reference avoid scenario: PolygonWorld1
%     waypoints{1} = [0.2 0; 7 0; 13 1; 15 0; 20 1; 25 -6; 16 -6];
%     waypoints{2} = [0.2 1; 10 -4; 15 0; 16 6; 8 8];
%     waypoints{3} = [0.2 2; 0.2 8; 20 1; 20 -8; 0 -8];
%     
%     x0{1} = [0;0;0];
%     x0{2} = [-1; 0; 0];
%     x0{3} = [1; 0; pi/2];
%     %scenario = MultiReferenceAvoidScenario(@SmoothDifferentialDriveVehicle, PolygonWorld1, waypoints, x0);
%     %scenario = MultiReferenceAvoidScenario(@SimpleUnicycleVehicle, PolygonWorld1, waypoints, x0);
% %     scenario = MultiReferenceAvoidScenario(@SimpleUnicycleVehicle, CorridorWorld, waypoints, x0);
%     scenario.tf = 30;
    
    
    % Create formation structure
    waypoints = [0.2 0; 12 0; 12 8; 20 8; 20 -6; 11.6 -6]; % Waypoints of the virtual leader
    Q = [ [0;0], [-1.5; 1.5], [-1.5; -1.5], [-3; 0], [2; 0]]; % Offsets for each agent
    
    x0{1} = [0.5;0;0;0;0];
    x0{2} = [0; 0; 0;0;0];
%     x0{3} = [-0.5; 0; 0;0;0];
%     x0{4} = [-0.25; 0; 0;0;0];
%     x0{5} = [-0.4; 0; 0;0;0];
    %scenario = MultiReferenceAvoidScenario(@SimpleUnicycleVehicle, PolygonWorld1, waypoints, x0, Q);
    %scenario = MultiReferenceAvoidScenario(@SimpleUnicycleVehicle, CorridorWorld, waypoints, x0, Q);
    scenario = MultiReferenceAvoidScenario(@BetterUnicycleVehicle, EmptyWorld, waypoints, x0, Q);
    %scenario = MultiReferenceAvoidScenario(@SimpleUnicycleVehicle, CorridorWorldStraight, waypoints, x0, Q);
    scenario.tf = 45;
    
%     % Wall following scenario
%     x0 = cell(1,0);
%     x0{1} = [0.5;0;0];
%     %x0{1} = [6;-4;0];
%     scenario = MultiWallFollowScenario(@SimpleUnicycleVehicle, CorridorWorld, x0);
%     scenario.tf = 60;
    
    % Run the scenario
    scenario.runScenario();
end

