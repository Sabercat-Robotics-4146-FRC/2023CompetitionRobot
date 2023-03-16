# Sabercat Robotics 2023 Robot Code Documentation

------------

<p align="center">
  <img src="/img/diagram.png" width="400" title="hover text">
</p>

## Drive
(Swerve Modules = SM1-SM4; Pigeon 2.0 = P)

The most fundamental aspect of our drive code is our swerve odometry, which keeps track of the robot's pose over time. Pose includes both the robot's translational and angular velocities. The gyroscope (Pigeon 2.0) collects data about the robot's angular velocity. By integrating this data with respect to time, the SwerveOdometry class onverts the robot's angular velocity into the robot's total change in angular heading. Given an initial position, we now know the robot's heading. SwerveOdometry then makes use of SwerveKinematics (which profiles the robot's drive motion) to then give velocity data. The end result is knowledge of the robot's translational and rotational velocity, aka pose!

The beauty of our drive code comes from the DriveSignal, an object which stores data on the robot's desired pose. Drive commands update the DriveSignal with the desired translational and angular velocities. By storing this data in a separate public object, it can be accessed instantaneously by updateModules() and updated instantaneously by drive() without interference. If we are following a trajectory (as in autonomous), we also update the DriveSignal periodically using SwerveOdometry data on the robot's position. It's all coming together...

The final piece in the puzzle is updateModules(). This method reads DriveSignal in order to generate a ChassisVelocity, which describes the motion of the entire robot. Using kinematics, it then converts ChassisVelocity into individual ModuleVelocities, describing the motion of each individual module. Then, it's as easy as setting the module.

DrivetrainSubsystem is registered as an Updatable so that this data is constantly being updated. Two final features relate to driver preference. The driver can toggle fieldOriented mode, or he/she can press a button to "zero" the gyroscope. This sets the robot's current heading as the default heading for future driving.



## Arm
(Rotation Motor = R; Extension Motor = E; Claw Motor = C)

Given the rapid-paced, highly variable nature of this year's game, we knew that arm controls had to combine reliability and efficiency. 

Sensor feedback was crucial for a reliable arm. We analyzed data relating to the arm's rotation from both the potentiometer and encoder. Because the potentiometer measured the rotation of the shaft closest to the actual arm, it gave us more precise data than the encoder, located at the driving shaft of the motor. Thus, it better reflected the arm's actual position. We also use the built-in Talon FX encoder for arm extension and a potentiometer for claw position*. Pre-recorded setpoints for each of these sensors allow us to trigger a command that moves the arm to the same position, every time. 

We also added protective functionality to the sensors. An upper and lower limit switch allow us to prevent harmful extension beyond the range of the arm. Additionally, when the lower limit switch is enabled (and the arm is fully retracted), we reset the extension encoder value to 0, ensuring accurate sensor data. We also configured a software limit on max. rotation with similar reasoning. A bonus feature is joystick rumbling to alert the drivers when the robot is driving with the arm either dangerously high or at risk of dragging on the ground.

PID control systems provide efficiency to our pickup and scoring methods. Proportional gain ensures we quickly reach our setpoint, and derivative gain reduces oscillations around that setpoint. With the help of a new Shuffleboard PID-testing method, we implemented our own PID in record time this season. We also sought to profile the arm motion to reduce fast, jerky motions observed in preliminary testing. To this effect, we added max. acceleration constraints and current limits.

* To detect whether the claw is gripping on an object, analyzing current draw proved more reliable than potentiometer data. The claw experiences additional torque when experiencing resistance from a gamepiece, which is detectable through current.



## Vision
(Limelight = L)

Limelight is our best friend when it comes to vision this year. The robot is configured to self-align itself with fiducials and retroreflective markers. Once it has detected a fiducial, the robot can identify its unique ID. PID control centers the robot left-right, allowing us to pinpoint the robot's exact position. From here, auto takes over.

Looking to the future, we hope to use an Intel RealSense camera to enhance our vision capabilites. The RealSense would also enable us to detect the difference between a cone and a cube upon pickup, as well as information about the orientation of the gamepiece. One of our largest present challenges is picking up and maneuvering a gamepiece with a non-upright orientation. Once we have identified the ideal orientation and position from which to pick up each gamepiece, we can automate this intake process. 



## Auto

We originally intended for this robot’s auto to follow a trajectory generated by a quintic hermite spline while running various autonomous commands before and after the path is followed. In order to generate this trajectory spline, we used WPILib’s Pathweaver tool. But, due to conflicts with the common package our team was using and WPILib’s trajectory package, we had to heavily modify Pathweaver in order to output data in a usable format. The modified Pathweaver outputs data correlating with specific waypoints marked in the program’s gamefield visual. These waypoint values were then used to generate a quintic hermite spline. This spline is then used to interpolate a path for the robot to follow. 

Unfortunately, in the last week before this competition, the spline generation and path following inexplicably stopped performing properly. We quickly modified our approach to using a combination of predetermined straight lines and rotations. While this method of autonomous movement is not as efficient as the previously mentioned method, it produces similar results, in terms of time and point value. The method of autonomously running subsystem commands remains the same: we run a command before and after the path is followed.

A final (and successful!) note to end on is our BalanceRobotCommand. This consists of a set trajectory to get the robot on top of the platform, followed by a PID sequence, using gyroscope feedback to level the robot. This method is highly reliable in both docking and engaging. 



## What's next?

Stay tuned to find out, we've got big things in store at Sabercat Robotics!
