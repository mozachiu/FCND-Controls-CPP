# Control of a 3D Quadrotor #

## The Goal of this Project ##

In the real world the flight controller is usually implemented in C or C++. So in this project we will implement your controller in C++. The code you write here can eventually be transferred to a real drone!

## Writeup ##

### Implemented body rate control in C++. ###
#### The controller should be a proportional controller on body rates to commanded moments. The controller should take into account the moments of inertia of the drone when calculating the commanded moments. ####

- Translate the commanded roll, pitch and yaw into desired rotational acceleration with proportional controller(pqrCmd - pqr).
```
momentCmd = VI * kpPQR * (pqrCmd - pqr);
```


### Implement roll pitch control in C++. ###
#### The controller should use the acceleration and thrust commands, in addition to the vehicle attitude to output a body rate command. The controller should account for the non-linear transformation from local accelerations to body rates. Note that the drone's mass should be accounted for when calculating the target angles. ####

- The equation below is used to calcuate body rate command in roll pitch controller.
<p align="center">
<img src="images/FCND02.png" width="400"/>
</p>

```
pqrCmd.x = (R21 * bc_dot.x - R11 * bc_dot.y) / R33;
pqrCmd.y = (R22 * bc_dot.x - R12 * bc_dot.y) / R33;

```
- Based on a desired global lateral acceleration and desired collective thrust.

### Implement altitude controller in C++. ###
#### The controller should use both the down position and the down velocity to command thrust. Ensure that the output value is indeed thrust (the drone's mass needs to be accounted for) and that the thrust includes the non-linear effects from non-zero roll/pitch angles.Additionally, the C++ altitude controller should contain an integrator to handle the weight non-idealities presented in scenario 4. ####

- Compute the integration part. "z_diff" is the distance between desired vertical position and current vertical position.
```
integratedAltitudeError += z_diff * dt;

```
- Compute feed-forward vertical acceleration "accelZCmd". "z_diff_dot" is the difference between desired vertical velocity and current vertical velocity.
```
accelZCmd += KiPosZ * integratedAltitudeError + kpVelZ * z_diff_dot;

```

- Finally compute the collective thrust command 
```
thrust = mass * ((float)CONST_GRAVITY - accelZCmd) / (float)R(2, 2);

```


### Implement lateral position control in C++. ###
#### The controller should use the local NE position and velocity to generate a commanded local acceleration. ####

- Initialize the returned desired acceleration to the feed-forward value.
- Compute desired velocity as below. "pos_Diff"  is the distance between desired  position and current position.
```

velCmd += kpPosXY * pos_Diff;

```
- Compute the commanded local acceleration as below. "velDiff_dot" is the difference between desired velocity and current velocity.
```
accelCmd += kpVelXY * velDiff_dot;

```
- Limit the maximum horizontal velocity and acceleration to maxSpeedXY and maxAccelXY

### Implement yaw control in C++. ###
#### The controller can be a linear/proportional heading controller to yaw rate commands (non-linear transformation not required). ####

- Compute yaw rate with multiply the proportional yaw constant kpYaw  by yaw delta which between commanded and observed yaw values.
```
yawRateCmd = kpYaw * fmodf(float(yawCmd - yaw), 2.*F_PI);
```

### Implement calculating the motor commands given commanded thrust and moments in C++. ###
#### The thrust and moments should be converted to the appropriate 4 different desired thrust forces for the moments. Ensure that the dimensions of the drone are properly accounted for when calculating thrust from moments. ####

- L : The arm length parameter L.kappa : the drag/thrust ratio kappa.
- The l is a distance between x-axis and propeller location.
- Using the following equations, we can get values of F1, F2, F3, and F4.

```
  Ft = F1+F2+F3+F4
  Fp = F1-F2+F3-F4
  Fq = F1+F2-F3-F4
  Fr = F1-F2-F3+F4
```

## Flight Evaluation ##
#### Your C++ controller is successfully able to fly the provided test trajectory and visually passes inspection of the scenarios leading up to the test trajectory. #####
Ensure that in each scenario the drone looks stable and performs the required task. Specifically check that the student's controller is able to handle the non-linearities of scenario 4 (all three drones in the scenario should be able to perform the required task with the same control gains used). 

#### Scenario 1 – Intro ####
- PASS: ABS(Quad.PosFollowErr) was less than 0.500000 for at least 0.800000 seconds
<p align="center">
<img src="images/FCND_S1.png" width="400"/>
</p>

#### Scenario 2 – Attitude Control ####
- PASS: ABS(Quad.Roll) was less than 0.025000 for at least 0.750000 seconds
- PASS: ABS(Quad.Omega.X) was less than 2.500000 for at least 0.750000 seconds
<p align="center">
<img src="images/FCND_S2.png" width="400"/>
</p>

#### Scenario 3 – Position Control ####
- PASS: ABS(Quad1.Pos.X) was less than 0.100000 for at least 1.250000 seconds
- PASS: ABS(Quad2.Pos.X) was less than 0.100000 for at least 1.250000 seconds
- PASS: ABS(Quad2.Yaw) was less than 0.100000 for at least 1.000000 seconds
<p align="center">
<img src="images/FCND_S3.png" width="400"/>
</p>

#### Scenario 4 – Nonidealities ####
- PASS: ABS(Quad1.PosFollowErr) was less than 0.100000 for at least 1.500000 seconds
- PASS: ABS(Quad2.PosFollowErr) was less than 0.100000 for at least 1.500000 seconds
- PASS: ABS(Quad3.PosFollowErr) was less than 0.100000 for at least 1.500000 seconds
<p align="center">
<img src="images/FCND_S4.png" width="400"/>
</p>

#### Scenario 5 – Trajectory Follow ####
- PASS: ABS(Quad2.PosFollowErr) was less than 0.250000 for at least 3.000000 seconds
<p align="center">
<img src="images/FCND_S5.png" width="400"/>
</p>
