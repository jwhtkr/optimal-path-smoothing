# Overview #
This code is still very much in development, so significant changes will occur in the short term. The goal is to provide a modular Matlab simulation capability for evaluating and demonstrating unmanned systems algorithms. 

## Running the code ##

The code is run using `launch.m`. Currently there are a variety of examples implemented, including

- Reference tracking
- Velocity tracking
- Vector field following
	- Single vector field
	- Summed vector fields (obstacle dependent)
	- Stitched vector fields (obstacle dependent)

Various types of 


## The basic lifecycle ##
The launch file instantiates a *Scenario* object and then runs the scenario by calling the *.runScenario()* method. The major objects of the simulation are:

- *Scenario*: Instantiation of `Scenario.m`
	-  *vehicle*: Instantiation of `Vehicle.m`
		-  *kinematics*: Instantiation of `VehicleKinematics.m`
		-  *sensor*: Insantiation of `RangeSensor.m`
	-  *world*: Instantiation of `PolygonWorld.m`


A **Scenario** object consists of a *vehicle* object and *world* object as well as parameters for plotting and simulation. When *.runScenario()* method is called, three things occur:

1. The state plots are initialized via a call to *obj.initializeStatePlot()*
2. The dynamics are simulated forward in time using Euler integration via a call to *obj.integrateEuler()*
3. The results of the simulation are plotted via a call to *obj.plotResults()*

The main function of the three called is *obj.integrateEuler()*. This function repeatedly does the following:

- Calls the *vehicle* object kinematic model to get the time derivative of the current state
- Updates the state
- Stores the state
- Plots the state

The  **vehicle** object is an instantiation of `Vehicle.m` which has three main elements:

- *kinematics*: Stores a kinematic model of the vehicle motion
- *sensor*: Stores a range sensor
- *control laws*: Implements control laws for (note: if the optional state vector is not specified then the control laws will use the vehicle's current state):
	- velocityControl(...): Inputs are desired velocities and optionally the state vector
	- pathControl(...): Inputs are the desired reference position and its first and second derivatives as well as an optional state vector
	- vectorFieldControl(...): Inputs are a handle to the vector field which takes in time, position, and orientation as well as an optional state vector

The **kinematics** object is an instantiation of `VehicleKinematics.m` which has two purposes:

1. Provide kinematic model for the motion
2. Plot the vehicle and state information

The **sensor** object is an instantiation of `RangeSensor.m` and has two purposes:

1. Calculate range measurements from the robot
2. Plot the range sensor measurements

Note that the number of range measurements and maximum distance are parameters that can be changed.

The **world** object is an instantiation of `PolygonWorld.m` and has three purposes:

1. Define a world from polygons
2. Determine if a point is inside a polygon
3. Plot the world as polygons

In addition to these main elements, there is also a *VectorField* object which can be used to define and plot a vector field to be followed.

# Implemented Code Overview #
We now use a bottom up approach to describe what objects have been implemented. 

## Kinematics ##
`VehicleKinematics.m` is the parent class for all of the kinematic models. The following abstract methods must be implemented by any child class:

-  *xdot = kinematics(obj, t, x, u)*: Takes in the current time, state, and input and calculate the time derivative of the state
-  *[v, w] = getVelocities(obj, t, x, u)*: Using knowledge of the time, state, and input this function calculates the translational velocity, v, and rotational velocity, w.

In addition to the abstract methods, `VehicleKinematics.m` implements several plotting functions which can be overridden by child objects, including

- *initializeStatePlot(...)*: Takes an axis handle and a state vector and plots the initial state as a polygon
- *plotState(...)*: Updates the polygon given the current state
- *getRobotPolygon(...)*: Returns a polygon defining the robot

There are multiple children of the *VehicleKinematics* object. These include:

- `SimpleUnicycle.m`: Implements the three state unicycle model (x,y,orientation) where (x,y) is the position where the inputs are direct control of (v, w) where (v, w) form the translational and rotational velocities
- `BetterUnicycle.m`: Implements the five state unicycle model (x, y, orientation, v, w) where input directly controls (a, alpha) where (a, alpha) are the translational and rotational accelerations
- `JerkUnicycle.m`: Implements a seven state unicycle model (x, y, orientation, v, w, a, alpha) where time derivatives of (a, alpha) are directly controlled
- `DifferentialDrive.m`: Implements a three state differential drive robot (x, y, orientation) where the wheel velocities (w_r, w_l) are directly controlled
- `SmoothDifferentialDrive.m`: Implements a five state differential drive robot (x, y, orientation, w_r, w_l) where the time derivatives of (w_r, w_l) are directly controlled
- `SimpleBicycle.m`: Implements a three state bicycle model (x, y, orientation, v, w) where the inputs are the steering angle, phi, and v.
- `ContinuousSteeringBicycle.m`: Implements a five state bicycle model (x, y, orientation, v, phi) where (a, phidot) are directly controlled and phidot is the time derivative of phi
- `ContinuousSteeringBicycle13_48.m`: Implements a six state bicycle model (x, y, orientation, v, phi, phidot) where (a phiddot) are directly controlled and phiddot is the time derivative of phidot.

## Sensors ##
`RangeSensor.m` is currently the only sensor implemented. It has two properties which define the sensing capability of the sensor:

- *n_lines*: Number of range measurements
- *max_dist*: Max distance of the range measurements

There are three major functions publically accessible on the *RangeSensor* object:

- *[xo, yo, dist_o] = getObstacleDetections(obj,q, th, world)*: Takes the position, orientation, and a polygon world as inputs and returns the (x,y) location to each detection as well as the distance to that detection
- *initializePlots(obj, ax)*: Takes in an axis handle and initializes the line and detection plots
- *plotMeasurements(obj, q, xo, yo)*: Takes in the current position of the vehicle and the (x,y) locations of obstacles and plots the line to each obstacle as well as a circle around the measurement.

## Vehicles ##
`Vehicle.m` is an abstract class for inherited to create vehicles. The constructor passes in the *kinematics* which is an instantiation of `VehicleKinematics.m` and then creates a *sensor* object which is an instantiation of `RangeSensor.m`. The *Vehicle* class has three abstract functions which define the control (note: if the optional state vector is not specified then the control laws will use the vehicle's current state. Also note that the output is a control input accepted by the *kinematics* model):

- *u = velocityControl(obj, vd, wd, varargin)*: Inputs are desired velocities and optionally the state vector
- *u = pathControl(obj, t, q_des, qd_des, qdd_des, varargin)*: Inputs are the desired reference position and its first and second derivatives as well as an optional state vector
- *u = vectorFieldControl(obj, t, g, varargin)*: Inputs are a handle to the vector field which takes in time, position, and orientation as well as an optional state vector

## Worlds ##

## Vector Fields ##

## Scenarios ##