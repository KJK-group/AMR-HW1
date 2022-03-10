# AMR Homework 1
All subtasks have their own launch file. Launching a task follows the standard form:

```sh
roslaunch husky_controllers exercise[problem-number].[task-number].launch <arguments>
```

## Problem 1
Each task has unique arguments described below
### Task 1.1
This task's launch takes the following arguments:
* `target_x`: x-coordinate of the control input.
* `target_y`: y-coordinate of the control input.
* `k_rho, k_alpha, k_beta`: controller gains.
These arguments are passed directly to the executable.

### Task 1.2
This task's launch takes the following arguments:
* `slope`: slope of first-order polynomial control input.
* `k_rho, k_alpha, k_beta`: controller gains.
These arguments are passed directly to the executable.

### Task 1.3
This task's launch takes the following arguments:
* `coefficient`: coefficient of the second-order polynomial control input.
* `k_rho, k_alpha, k_beta`: controller gains.
These arguments are passed directly to the executable.

## Problem 2
The launch files for the subtasks in this problem, don't take any arguments.
