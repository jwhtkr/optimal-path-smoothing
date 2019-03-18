function launch()
    % Clear previous work
    close all;
    clear;
    clc;
    
    % Configure file path
    addpath Dependencies\intersections
    addpath Scenarios
    addpath Sensors
    addpath Vehicle
    addpath Vehicle\Kinematics
    addpath Vehicle\LowLevelControl
    addpath World
    
    % Create scenario
    scenario = ReferenceTrackingUnicycleScenario();
    %scenario = VelocityTrackingUnicycleScenario();
    
    
    % Run the scenario
    scenario.runScenario();
    

end

