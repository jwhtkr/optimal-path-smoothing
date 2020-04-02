function launchMPCDiffFlat()
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
    goalpoints = [0 -1.5]; %; 13 -1.5; 13 10; 18.25 10; 18.25 -4; 0 -4]; % Waypoints of the virtual leader
    
    x0{1} = [0.5;0;0; 0.1;0];
    %x0{1} = [0.5;0;0];

    scenario = MultiMPCGoToGoal(EmptyWorld, goalpoints, x0);
    
    scenario.tf = 50;
    
    % Run the scenario
    scenario.runScenario();
end

