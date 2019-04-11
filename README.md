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

Starts the plot and then you need to press enter ...


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

The  **vehicle** object is an instantiation of `Vehicle.m` that has three main elements:

- *kinematics*: Stores a kinematic model of the vehicle motion
- *sensor*: Stores a range sensor
- *control laws*: Implements control laws for (note: if the optional state vector is not specified then the control laws will use the vehicle's current state):
	- velocityControl(...): Inputs are desired velocities and optionally the state vector
	- pathControl(...): Inputs are the desired reference position and its first and second derivatives as well as an optional state vector
	- vectorFieldControl(...): Inputs are a handle to the vector field that takes in time, position, and orientation as well as an optional state vector

The **kinematics** object is an instantiation of `VehicleKinematics.m` that has two purposes:

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

In addition to these main elements, there is also a *VectorField* object that can be used to define and plot a vector field to be followed.

# Implemented Code Overview #
We now use a bottom up approach to describe what objects have been implemented. 

## Kinematics ##
`VehicleKinematics.m` is the parent class for all of the kinematic models. The following abstract methods must be implemented by any child class:

-  *xdot = kinematics(obj, t, x, u)*: Takes in the current time, state, and input and calculate the time derivative of the state
-  *[v, w] = getVelocities(obj, t, x, u)*: Using knowledge of the time, state, and input this function calculates the translational velocity, v, and rotational velocity, w.

In addition to the abstract methods, `VehicleKinematics.m` implements several plotting functions that can be overridden by child objects, including

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
`RangeSensor.m` is currently the only sensor implemented. It has two properties that define the sensing capability of the sensor:

- *n_lines*: Number of range measurements
- *max_dist*: Max distance of the range measurements

There are three major functions publically accessible on the *RangeSensor* object:

- *[xo, yo, dist_o] = getObstacleDetections(obj,q, th, world)*: Takes the position, orientation, and a polygon world as inputs and returns the (x,y) location to each detection as well as the distance to that detection
- *initializePlots(obj, ax)*: Takes in an axis handle and initializes the line and detection plots
- *plotMeasurements(obj, q, xo, yo)*: Takes in the current position of the vehicle and the (x,y) locations of obstacles and plots the line to each obstacle as well as a circle around the measurement.

## Vehicles ##
`Vehicle.m` is an abstract class for inherited to create vehicles. The constructor passes in the *kinematics*, which is an instantiation of `VehicleKinematics.m`, and then creates a *sensor* object, which is an instantiation of `RangeSensor.m`. The *Vehicle* class has three abstract functions that define the control (note: if the optional state vector is not specified then the control laws will use the vehicle's current state. Also note that the output is a control input accepted by the *kinematics* model):

- *u = velocityControl(obj, vd, wd, varargin)*: Inputs are desired velocities and optionally the state vector
- *u = pathControl(obj, t, q_des, qd_des, qdd_des, varargin)*: Inputs are the desired reference position and its first and second derivatives as well as an optional state vector
- *u = vectorFieldControl(obj, t, g, control_type, varargin)*: Inputs are a handle to the vector field that takes in time, position, and orientation as well as the type of vector field control to be used and an optional state vector. The *control_type* input is of type `VECTOR_FOLLOWING_TYPE`. `VECTOR_FOLLOWING_TYPE` is an enumerated class with the following three possibilities:
	-  *VELOCITY*: Denotes a control law that converts the vector *g* into desired translational and rotational velcoities	
	-  *ORIENTATION*: Denotes a control law that converts the vector *g* into a desired orientation and translational velocity 
	-  *POINT*: Denotes a control law that uses the vector *g* as a desired velocity for a point in front of the vehicle

In addition to the abstract methods, there are three methods that are implemented in `Vehicle.m`:

- *[xo, yo, dist] = getObstacleDetections(obj, world)*: Accesses the range sensor to get the obstacle detections given the robots current position in the world. The *world* input is a type *PolygonWorld*. The outputs are the *(x,y)* position of the obstacles and the distance to the obstacle from the robot's position.
- *initializePlots(obj, ax)*: Initializes a plot of the vehicle and sensor measurements given a handle to a figure axis.
- *plotVehicle(obj)*: Updates the plots of the vehicle and the sensor measurements

Several *Vehicle* objects have been defined:

- `BetterUnicycleVehicle.m`: Uses the *BetterUnicycle* kinematic model and implements all of the control laws for that model.
- `SimpleUnicycleVehicle.m`: Uses the *SimpleUnicycle* kinematic model and implements all of the control laws for that model.
- `SmoothDifferentialDriveVehicle.m`: Uses the *SmoothDifferentialDrive* kinematic model and implements all of the control laws for that model.

## Worlds ##
`PolygonWorld.m` is a class that takes in an arbitrary number of polygons and uses them to define a world. It has two methods of import:

1. *plotWorld(obj, ax)*: Takes a figure axis handle and plots the polygons that define the world.
2. *result = insideObstacles(obj, x, y)*: Determines if a point, *(x,y)*, is inside a polygon. The output, *result*, is assigned true if the points is inside an obstacle, false otherwise.

There are currently two worlds that inherit *PolygonWorld*:

- `EmptyWorld.m`: A world without any obstacles
- `PolygonWorld1.m`: A world with three obstacles

## Vector Fields ##
`VectorField.m` is an abstract class that is used to evaluate and plot vector fields. It has a single abstract method:

- *g = getVector(obj, t, x, th)*: A function that returns a 2D vector given the current time, position, and orientation.

In addition to the abstract vector function, the *VectorField* object has functions for plotting the vector field:

- *h = plotVectorField(obj, t)*: Plots the vector field at the given time

There are multiple children classes of the *VectorField* object that each implement a different vector field:

- `AvoidObstacle.m`: Implements a basic vector field pointing away from an obstacle point. Several parameters defining the vector field can be set:
	- *S*: Sphere of influence
	- *R*: Radius of max effect
	- *x_o*: Obstacle 2D position
	- *v_max*: Maximum magnitude produced by the vector field
- `GoToGoalField.m`: Basic vector field pointing to a desired goal point. Several parameters define the vector field:
	- *x_g*: Goal position
    - *v_max*: Maximum magnitude produced by the vector field
    - *sig*: Effects the convergence to zero velocity through 1-exp(-d^2/sig^2)
- `LineVectorField.m`: Vector field causing vehicle to move to a line
	- Parameters defining field
		- *x_l*: A point on the line
	    - *psi_l*: The orientation of the line
	    - *slope*: Slope of the sigmoid function defining the line
	    - *v_d*: Desired velocity / length of the vector field line
    - Additional interface function
	    - *setLineParameters(obj, x_l, psi_l)*: Sets the parameters defining the line (point on line and orientation of line)
- `OrbitField.m`: Vector field that will move vehicle into an orbit. Parameters defining the field:
	- *x_c*: 2D center position of the orbit
    - *rad*: radius of the orbit
    - *w*: frequency of the orbit
    - *k_conv*: gain on convergence to the orbit
- `OrbitAvoidField.m`:  Creates a vector field moving the vehicle around an obstacle position using an orbit. Parameters defining field: 
	- *x_o*: 2D center position of the orbit (typically updated to use the obstacle position as the center of the orbit)
    - *rad*: radius of the orbit
    - *k_conv*: gain on convergence to the orbit
    - *v_d*: Desired speed of the orbit (calculated using w*rad)
    - *S*: Sphere of influence
- `SummedFields.m`: Sums together multiple fields to get the contribution. Parameters defining field:
	- *fields*: A cell structure where each element is an instance of the VectorField object
    - *weights*: A vector of dimension n_fields that provides a weight for each field
    - *v_max*: Maximum magnitude produced by the vector field

## Scenarios ##
`Scenario.m` is an abstract class whose variables and methods define a situation to be simulated and also simulate that situation. The *Scenario* object consists of five major elements:
- Objects that define the world and vehicle
- Definition of movement
- Definition of period of performance of the simulation
- Visualization of the simulation
- Five non-abstracted methods

Two **main objects** define the the elements in the scenario's world:

1. *vehicle* object - Defines the control and movement of the vehicle. An instantiation of the `Vehicle.m` class 
2. *world* object - Defines the world that the vehicle is to be operating in.

The *Scenario* object has one abstract method that is used to **define the movement** of the vehicle.

- *u = control(obj, t, x)*: Determines what control law to employ and the desired values for that control law.

A major function of the *Scenario* object is to simulate the vehicle forward in time. Several parameters define the **period of performance** for the simulation:

- *t0*: Initial time of simulation
- *dt*: Simulation step size
- *tf*: Final time of simulation

There are also two parameters that effect the **visualization of the simulation**:

- *plot_during_sim*: true => plot while simulating (requires euler integration), false => no plotting during simulation (faster to see the final results)
- *T*: Plotting period - The plotting functions will only be called every *T* seconds. This helps to have faster simulation by not spending so many resources on visualizing the vehicle and world at every step of the simulation.

The *Scenario* object has five **main methods**:

- *runScenario()*: Method for running the simulation. It performs three tasks:
	- The state plots are initialized via a call to *obj.initializeStatePlot()*
	- The dynamics are simulated forward in time using Euler integration via a call to *obj.integrateEuler()*
	- The results of the simulation are plotted via a call to *obj.plotResults()*
- *integrateEuler()*: This method simulates the state of the vehicle from time *t0* to time *tf*. It repeatedly does the following:
	- Calls the *control(...)* method to calculate the control input at the current state
	- Calls the *vehicle* object kinematic model to get the time derivative of the current state
	- Updates the state
	- Stores the state
	- Plots the state 
- *initializeStatePlot()*: Initializes the plotting of the world and the vehicle
- *plotState(t)*: Updates the plot of the vehicle and its trajectory at time *t*
- *plotResults()*: Plots the velocity profiles and control over the simulation time


### Scenario Instantiations ###
There are a variety of objects that implement the abstract *Scenario* class. These objects largely differ in how they define the world, the control function that they call on the vehicle, and the plotting. These implementations are generic in that they accept any *Vehicle* object. There are three fundamental implementations:

- `ReferenceTrackingScenario.m`: Has a vehicle follow a sinusoidal path.
	- World: *EmptyWorld*
	- Control: *pathControl(...)*
	- Plotting: It plots the reference trajectory in addition to the vehicle during simulation. The *plotResults(...)* function plots the position states and desired positions as well as the overall error vs time.
- `VelocityTrackingScenario.m`: Has a vehicle track desired velocities over time.
	- World: *EmptyWorld*
	- Control: *velocityControl(...)*
	- Plotting: The *plotResults(...)* function plots the velocities and desired velocities over time.
- `VectorFieldScenario.m`: Controls a vehicle to follow a given vector field
	- World: Passed into constructor
	- Control: *vectorFieldControl(...)* where the vector field is defined by a *VectorField* object and the vector-field control type is specified by an instance of `VECTOR_FOLLOWING_TYPE`, both passed into the constructor of the *VectorFieldScenario* object.
	- Plotting: *initializeStatePlot(...)* also plots the vector field

In addition to these fundamental implementations, there are a number of implementations that inherit the *VectorFieldScenario* object. They all allow any *Vehicle* object as well as any `VECTOR_FOLLOWING_TYPE` to be used as a controller. They all currently use the *PolygonWorld1* world and update the plot of the vector field in real time, but differ in the vector field used to define the control. They include:

- `CombinedGoToGoalVectorScenario.m`: Simple go-to-goal with simple obstacle avoidance
	- Field: Sums a `GoToGoalField.m` with an `AvoidObstacle.m` for each sensor measurement.
- `CombinedGoToGoalOrbitAvoidScenario.m`: Simple go-to-goal with orbit obstacle avoidance 
	- Field: Sums a `GoToGoalField.m` with an `OrbitAvoidField.m` for each sensor measurement.
- `CombinedGoToGoalOrbitAvoidWithBarrierScenario.m`: Simple go-to-goal with orbit obstacle avoidance and simple obstacle avoidance (used for ensuring obstacle barrier)
	- Field: Sums a `GoToGoalField.m` with an `OrbitAvoidField.m` and an `AvoidObstacle.m` for each sensor measurement.
- `SwithingLineScenario.m`: Uses a simple distance based switching function to switch between line vector fields
	- Field: `LineVectorField.m` that is updated once the vehicle gets sufficiently close to the goal point
- `SwithingLineScenarioObstacleAvoid.m`: Uses a simple distance based switching function to switch between line vector fields while also using an orbit obstacle avoidance field
	- Field: `LineVectorField.m` that is updated once the vehicle gets sufficiently close to the goal point summed with `OrbitAvoidField.m` for each sensor measurement.
- `SwithingLineScenarioObstacleAvoidBetterSwitch`: Uses a half-plane to define the switching between line vector fields while also using an orbit obstacle avoidance field
	- Field: `LineVectorField.m` that is updated once the vehicle passes the a half plane summed with `OrbitAvoidField.m` for each sensor measurement. 


