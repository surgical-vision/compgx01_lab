# COMPGX01 - Inverse Kinematics

In this lab, we will be looking at inverse kinematics using KDL, Kinematics and Dynamics Library. The package contains a Youbot Class which will be used as a base class, and an implemented version using KDL.

##  Setting up a ROS bag with test cases
We need to make something to test the inverse kinematics solvers with. ROS bags are a method of holding messages. 
The node will use the YoubotKDL Class to do the forward kinematics and publish the current transform. 
You will write:
  * A function to publish random (but within it's limits) joint values to the youbot
  * A function to write joint vales and the current frame in the ROS bag

## Test the solver
The YoubotKDL class has the one of the inverse kinematic solvers implemeted which will be used to develop the testing framework.
You will make a node that uses:
  * YoubotKDL class
  * ROS bag

The node will:
  * Load a desired transform from the ros bag
  * Find the required joint values using the inverse_kinematics function
  * Move the robot to the joint values and compare with the current position

## Jacobian based solver
Extend the YoubotKDL class with your own jacobian based solver. The YoubotKDL class has the `get_jacobian` funciton implemented, use this to implement your own `inverse_kinematics_jac` function in the class.
