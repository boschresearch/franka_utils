# Utilities for Franka

This repo contains standalone programs to interface with the Franka via FCI.

## Compute and set end-effector properties from URDF

Run the `compute_lumped_inertia` executable with the URDF and the IP:
```sh
compute_lumped_inertia $URDF $IP
```
On ROS 2, this can be combined with the xacro command and the `franka_description` package:
```sh
compute_lumped_inertia "$(xacro $(ros2 pkg prefix franka_description)/share/franka_description/robots/panda_arm.urdf.xacro)" 172.16.0.2
```

This will compute the end-effector properties from the robot description and print those values on stdout and, if the robot is reachable, set those values in the Franka controller. This can be verified by running the command twice. The second time the `Franka current state:` will list the new properties in the yaml string.

## F/T thresholds

The executable `set_collision_behaviour` sets the torque and force threshold via the `franka::Robot::setCollisionBehavior` method. It uses a ratio to scale the default F/T thresholds. To set the threshold to 5 times the default:
```sh
# set_collision_behaviour $ROBOT_IP $THRESHOLD_SCALE
set_collision_behaviour 172.16.0.2 5.0
```
This is useful when interacting with stiff objects and expecting large than default contact forces.
