# PID Longitudinal Controller

## Purpose / Use cases

The longitudinal_controller computes the target acceleration to achieve the target velocity set at each point of the target trajectory using a feed-forward/back control.

It also contains a slope force correction that takes into account road slope information, and a delay compensation function.
It is assumed that the target acceleration calculated here will be properly realized by the vehicle interface.

Note that the use of this module is not mandatory for Autoware if the vehicle supports the "target speed" interface.

## Design / Inner-workings / Algorithms

### States

This module has four state transitions as shown below in order to handle special processing in a specific situation.

- **DRIVE**
  - Executes target velocity tracking by PID control.
  - It also applies the delay compensation and slope compensation.
- **STOPPING**
  - Controls the motion just before stopping.
  - Special sequence is performed to achieve accurate and smooth stopping.
- **STOPPED**
  - Performs operations in the stopped state (e.g. brake hold)
- **EMERGENCY**.
  - Enters an emergency state when certain conditions are met (e.g., when the vehicle has crossed a certain distance of a stop line).
  - The recovery condition (whether or not to keep emergency state until the vehicle completely stops) or the deceleration in the emergency state are defined by parameters.

The state transition diagram is shown below.

![LongitudinalControllerStateTransition](./media/LongitudinalControllerStateTransition.drawio.svg)

### Logics

#### Control Block Diagram

![LongitudinalControllerDiagram](./media/LongitudinalControllerDiagram.drawio.svg)

#### FeedForward (FF)

The reference acceleration set in the trajectory and slope compensation terms are output as a feedforward. Under ideal conditions with no modeling error, this FF term alone should be sufficient for velocity tracking.

Tracking errors causing modeling or discretization errors are removed by the feedback control (now using PID).

##### Brake keeping

From the viewpoint of ride comfort, stopping with 0 acceleration is important because it reduces the impact of braking. However, if the target acceleration when stopping is 0, the vehicle may cross over the stop line or accelerate a little in front of the stop line due to vehicle model error or gradient estimation error.

For reliable stopping, the target acceleration calculated by the FeedForward system is limited to a negative acceleration when stopping.

![BrakeKeepingDiagram](./media/BrakeKeeping.drawio.svg)

#### Slope compensation

Based on the slope information, a compensation term is added to the target acceleration.

There are two sources of the slope information, which can be switched by a parameter.

- Pitch of the estimated ego-pose (default)
  - Calculates the current slope from the pitch angle of the estimated ego-pose
  - Pros: Easily available
  - Cons: Cannot extract accurate slope information due to the influence of vehicle vibration.
- Z coordinate on the trajectory
  - Calculates the road slope from the difference of z-coordinates between the front and rear wheel positions in the target trajectory
  - Pros: More accurate than pitch information, if the z-coordinates of the route are properly maintained
  - Pros: Can be used in combination with delay compensation (not yet implemented)
  - Cons: z-coordinates of high-precision map is needed.
  - Cons: Does not support free space planning (for now)

**Notation:** This function works correctly only in a vehicle system that does not have acceleration feedback in the low-level control system.

This compensation adds gravity correction to the target acceleration, resulting in an output value that is no longer equal to the target acceleration that the autonomous driving system desires. Therefore, it conflicts with the role of the acceleration feedback in the low-level controller.
For instance, if the vehicle is attempting to start with an acceleration of `1.0 m/s^2` and a gravity correction of `-1.0 m/s^2` is applied, the output value will be `0`. If this output value is mistakenly treated as the target acceleration, the vehicle will not start.

A suitable example of a vehicle system for the slope compensation function is one in which the output acceleration from the longitudinal_controller is converted into target accel/brake pedal input without any feedbacks. In this case, the output acceleration is just used as a feedforward term to calculate the target pedal, and hence the issue mentioned above does not arise.

Note: The angle of the slope is defined as positive for an uphill slope, while the pitch angle of the ego pose is defined as negative when facing upward. They have an opposite definition.

![slope_definition](./media/slope_definition.drawio.svg)

#### PID control

For deviations that cannot be handled by FeedForward control, such as model errors, PID control is used to construct a feedback system.

This PID control calculates the target acceleration from the deviation between the current ego-velocity and the target velocity.

This PID logic has a maximum value for the output of each term. This is to prevent the following:

- Large integral terms may cause unintended behavior by users.
- Unintended noise may cause the output of the derivative term to be very large.

Note: by default, the integral term in the control system is not accumulated when the vehicle is stationary. This precautionary measure aims to prevent unintended accumulation of the integral term in scenarios where Autoware assumes the vehicle is engaged, but an external system has immobilized the vehicle to initiate startup procedures.

However, certain situations may arise, such as when the vehicle encounters a depression in the road surface during startup or if the slope compensation is inaccurately estimated (lower than necessary), leading to a failure to initiate motion. To address these scenarios, it is possible to activate error integration even when the vehicle is at rest by setting the `enable_integration_at_low_speed` parameter to true.

When `enable_integration_at_low_speed` is set to true, the PID controller will initiate integration of the acceleration error after a specified duration defined by the `time_threshold_before_pid_integration` parameter has elapsed without the vehicle surpassing a minimum velocity set by the `current_vel_threshold_pid_integration` parameter.

The presence of the `time_threshold_before_pid_integration` parameter is important for practical PID tuning. Integrating the error when the vehicle is stationary or at low speed can complicate PID tuning. This parameter effectively introduces a delay before the integral part becomes active, preventing it from kicking in immediately. This delay allows for more controlled and effective tuning of the PID controller.

At present, PID control is implemented from the viewpoint of trade-off between development/maintenance cost and performance.
This may be replaced by a higher performance controller (adaptive control or robust control) in future development.

#### Time delay compensation

At high speeds, the delay of actuator systems such as gas pedals and brakes has a significant impact on driving accuracy.
Depending on the actuating principle of the vehicle, the mechanism that physically controls the gas pedal and brake typically has a delay of about a hundred millisecond.

In this controller, the predicted ego-velocity and the target velocity after the delay time are calculated and used for the feedback to address the time delay problem.

### Slope compensation

Based on the slope information, a compensation term is added to the target acceleration.

There are two sources of the slope information, which can be switched by a parameter.

- Pitch of the estimated ego-pose (default)
  - Calculates the current slope from the pitch angle of the estimated ego-pose
  - Pros: Easily available
  - Cons: Cannot extract accurate slope information due to the influence of vehicle vibration.
- Z coordinate on the trajectory
  - Calculates the road slope from the difference of z-coordinates between the front and rear wheel positions in the target trajectory
  - Pros: More accurate than pitch information, if the z-coordinates of the route are properly maintained
  - Pros: Can be used in combination with delay compensation (not yet implemented)
  - Cons: z-coordinates of high-precision map is needed.
  - Cons: Does not support free space planning (for now)

## Assumptions / Known limits

1. Smoothed target velocity and its acceleration shall be set in the trajectory
   1. The velocity command is not smoothed inside the controller (only noise may be removed).
   2. For step-like target signal, tracking is performed as fast as possible.
2. The vehicle velocity must be an appropriate value
   1. The ego-velocity must be a signed-value corresponding to the forward/backward direction
   2. The ego-velocity should be given with appropriate noise processing.
   3. If there is a large amount of noise in the ego-velocity, the tracking performance will be significantly reduced.
3. The output of this controller must be achieved by later modules (e.g. vehicle interface).
   1. If the vehicle interface does not have the target velocity or acceleration interface (e.g., the vehicle only has a gas pedal and brake interface), an appropriate conversion must be done after this controller.

## Inputs / Outputs / API

### Input

Set the following from the [controller_node](../trajectory_follower_node/README.md)

- `autoware_auto_planning_msgs/Trajectory` : reference trajectory to follow.
- `nav_msgs/Odometry`: current odometry

### Output

Return LongitudinalOutput which contains the following to the controller node

- `autoware_auto_control_msgs/LongitudinalCommand`: command to control the longitudinal motion of the vehicle. It contains the target velocity and target acceleration.
- LongitudinalSyncData
  - velocity convergence(currently not used)

### PIDController class

The `PIDController` class is straightforward to use.
First, gains and limits must be set (using `setGains()` and `setLimits()`) for the proportional (P), integral (I), and derivative (D) components.
Then, the velocity can be calculated by providing the current error and time step duration to the `calculate()` function.

## Parameter description

The default parameters defined in `param/lateral_controller_defaults.param.yaml` are adjusted to the
AutonomouStuff Lexus RX 450h for under 40 km/h driving.

> **STOPPING Parameter (smooth stop)** is enabled if `enable_smooth_stop` is true.
> In smooth stop, strong acceleration (`strong_acc`) will be output first to decrease the ego velocity.
> Then weak acceleration (`weak_acc`) will be output to stop smoothly by decreasing the ego jerk.
> If the ego does not stop in a certain time or some-meter over the stop point, weak acceleration to stop right (`weak_stop_acc`) now will be output.
> If the ego is still running, strong acceleration (`strong_stop_acc`) to stop right now will be output.

> **The `STOPPED` state** assumes that the vehicle is completely stopped with the brakes fully applied.
> Therefore, `stopped_acc` should be set to a value that allows the vehicle to apply the strongest possible brake.
> If `stopped_acc` is not sufficiently low, there is a possibility of sliding down on steep slopes.

{{ json_to_markdown("control/pid_longitudinal_controller/schema/pid_longitudinal_controller.schema.json") }}

## References / External links

## Future extensions / Unimplemented parts

## Related issues
