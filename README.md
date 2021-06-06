# movement_utils

Wraps existing ROS movement for more straightforward actions.

The `movement_wrapper.py` node breaks movement into 'steps': rather
than being continuous movements, they are discrete individual actions.
For broader funcitonality, `movement_wrapper.launch` launches this
node alongside `default.yaml`, the default yaml to control velocity,
distance, and allowable error per step.
