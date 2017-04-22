# Unscented Kalman Filter
## Udacity - Term 2 - Project 2

This project involved the construction of an Unscented Kalman filter, to read in RADAR and LIDAR data, to determine the position, velocity, yaw and yaw rate of a tracked object - in this case a bicyle. The motion model in use is a constant turn rate and velocity magnitude (CTRV) model. At each time step, the estimated state is compared to the ground-truth value, in order to produce a Root Mean Squared Error (RMSE) of the positive (in x and y) and the velocity (in x and y).

This project required the tuning of the longitudinal acceleration noise and yaw acceleration noise. Based on some rules of thumb, it is recommended to estimate the maximum acceleration of the tracked object, and use half the value as process noise. So, a bike is estimated to accelerate at 2m.s<sup>-2</sup> (fast bicycle), so I have set process noise to 1m.s<sup>-2</sup>. The yaw acceleration noise is set to 1 rad.s<sup>-2</sup>. These are probably not the most optimal but they give "reasonable" results, as witnessed by utilising the "plot.py" script to visualise the results - credit to https://github.com/hq-jiang/CarND-Unscented-Kalman-Filter-Project/blob/master/read_csv.py - with a few changes to the code.

The project was built in VS 2017, however, the VS project files have been removed, and the files have been tested in Windows 10 using cmake, minGW and make versions as follows:
* cmake v 3.7.2
* make v 3.81
* minGW c++ (GCC) v5.3.0

The user may enter the following commands in cmd to build, in Windows:
* `mkdir build && cd build`
* `cmake .. -G "Unix Makefiles" && make`
* `UnscentedKF.exe ../data/obj_pose-laser-radar-synthetic-input.txt output.txt`

## Response to rubric requirements
### Compiling
* Requirement: Code should compile without errors with `cmake` and `make`
* Response: Code compiles without error.

### Accuracy
The following table shows the RMSE requirements and results for "obj_pose-laser-radar-synthetic-input.txt". Out of interest, data was also collected for only LIDAR and only RADAR sensors, by setting flags (use_laser_, use_radar_ lines 16, 19) in the ukp source file, which has to be done manually. The results for Lidar+Radar are below the requirements, and the velocity is more accurate than the extended kalman filter in P1, due to the use of the CTRV model instead of CV model, and improved non-linear capability of UKF.

| Requirements (<=) | L + R |  L only | R only |
| ------------- | ------------- | ------------- | ------------- |
| px = 0.09 | px = 0.06 | px = 0.10 | px = 0.15 |
| py = 0.10 | py = 0.08 | py = 0.10 | py = 0.22 |
| vx = 0.40 | vx = 0.33 | vx = 0.61 | vx = 0.36 |
| vy = 0.30 | vy = 0.22 | vy = 0.26 | vy = 0.31 |

### Follows the correct algorithm
Criteria: Your Sensor Fusion algorithm follows the general processing flow as taught in the preceding lessons.
* Response: The steps have been implemented according to the provided template and lessons.

Criteria: Your Kalman Filter algorithm handles the first measurements appropriately.
* Response: The first measured is used to initialise, and deals with px==0 and py==0

Criteria: Your Kalman Filter algorithm first predicts then updates.
* Response: Yes, it performs prediction before the update step with Kalman (lidar) or Extended Kalman (radar)

Criteria: Your Kalman Filter can handle radar and lidar measurements.
* Response: It handles both adequately. It was tested with the kalman-tracker as well, and proved successful.

### Code efficiency
Criteria: Your algorithm should avoid unnecessary calculations.
* Response: I believe the majority of my code was efficient. I tried to make some changes to improve the efficiency of the matrix calcs (i.e. storing transpose, or inverse where it saved a calculation). Also, instead of using the more expensive method of augmenting sigma points for LIDAR measurements, I utilised a kalman filter to update the state and covariance instead, as the process is linear, allowing this use.

