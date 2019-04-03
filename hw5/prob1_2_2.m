function prob1_2_2()
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
    scenario = ReferenceTrackingScenario(SmoothDifferentialDriveVehicle);
    
    % Run the scenario
    scenario.runScenario();
end

