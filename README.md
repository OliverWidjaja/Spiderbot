Code Working Structure:

VESCCommands.py stores all the command to control VESC, including moving motor and resetting to zero position.

RobotTrajectory.py handles robot trajectory, e.g. suspension operation, traversal, and desuspension operation of robot.

MotiveSocket.py receives with NatNet packets and organizes packets into easily-called format.

DroneTracking.py will handle autonomous drone mission navigation by calling functions based on real-time streamed packets.
