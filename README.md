# Unscented Kalman Filter
## Udacity - Term 2 - Project 2

This project involved the construction of an Unscented Kalman filter, to read in RADAR and LIDAR data, to determine the relative position and speed of a tracked object. At each time step, the estimated state is compared to the ground-truth value, in order to produce a Root Mean Squared Error (RMSE) of the positive (in x and y) and the velocity (in x and y).

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
The following table shows the RMSE requirements and results for "sample-laser-radar-measurement-data-1.txt"

| Requirements (<=) | Outcomes |
| ------------- | ------------- |
| px = 0.08 | px = 0.065 |
| py = 0.08 | py = 0.060 |
| vx = 0.60 | vx = 0.533 |
| vy = 0.60 | vy = 0.544 |

The following table shows the RMSE requirements and results for "sample-laser-radar-measurement-data-2.txt"

| Requirements (<=) | Outcomes |
| ------------- | ------------- |
| px = 0.20 | px = 0.185 |
| py = 0.20 | py = 0.190 |
| vx = 0.50 | vx = 0.477 |
| vy = 0.85 | vy = 0.804 |

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
* Response: The majority of the code is efficient. The normalisation step to ensure the angle is between -pi and pi has the potential to loop many times for a large angle. There may be a more efficient way to normalise this, rather than adding or subtracting 2PI until the angle meets the criteria.

