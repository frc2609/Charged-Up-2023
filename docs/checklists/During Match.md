# Issues/Solutions During a Match

## Driver
* Robot doesn't drive in a field-oriented manner
    * Reset yaw
    * Operator check rotation and drive motors

## Operator
* Arm does not reach setpoint (Arm absolute encoder and arm NEO encoder angles do not match)
    * Move arm to stow position using manual control
    * Reset arm position (`Select` button)