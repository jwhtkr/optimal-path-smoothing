function launchRefAvoidSingleVehicle()
    % Clear previous work
    close all;
    clear;
    clc;
    
    % Configure file path
    addpath Agents
    addpath ComplexBehaviors\WallFollowing
    addpath Dependencies
    addpath Dependencies\distinguishable_colors
    addpath Dependencies\intersections
    addpath Plotters
    addpath Scenarios
    addpath Scenarios\MultiAgentScenarios
    addpath Sensors
    addpath TrajectoryPlanners
    addpath TrajectoryPlanners\Clothoid
    addpath VectorFields
    addpath Vehicle
    addpath Vehicle\Kinematics
    addpath World
    
    % Create formation structure
    %waypoints = [0.2 0; 12 0; 12 8; 20 8; 20 -6; 0 -6]; % Waypoints of the virtual leader
    %waypoints = [0 -1.5; 13 -1.5; 13 5.5; 19 5.5; 19 -5; 0 -5]; % Waypoints of the virtual leader
    waypoints = [0 -1.5; 13 -1.5; 13 10; 18.25 10; 18.25 -4; 0 -4]; % Waypoints of the virtual leader
    %waypoints = [0 -2; 5 3; 11 -2; 15 10; 22 10; 16 0; 18 -4; 15 -6; 10 -2]; % Waypoints of the virtual leader
    Q = [ [0;0], [-1.5; 1.5], [-1.5; -1.5], [-3; 0], [2; 0]]; % Offsets for each agent
    
    x0{1} = [0.5;0;0; 0;0];
    %x0{1} = [0.5;0;0];

    scenario = MultiReferenceAvoidScenario(@BetterUnicycleVehicle, CorridorWorld, waypoints, x0, Q);
    %scenario = MultiReferenceAvoidScenario(@SimpleUnicycleVehicle, CorridorWorld, waypoints, x0, Q);
    %scenario = MultiWallFollowScenario(@BetterUnicycleVehicle, CorridorWorld, x0);
    
    scenario.tf = 50;
    
    % Run the scenario
    scenario.runScenario();
end

