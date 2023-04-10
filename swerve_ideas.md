# Swerve

-> Can I automatically decrease reduction ratio as speed increases?
-> If (error) bigger, make gear ratio lower to get there faster -> how implement

## Functions
* getMaxSpeed(double gearRatio) ...
* 

## hi

How do I control this:
* As we increase in speed, decrease the ratio
**TARGET GEAR RATIO**
-> function that converts boost/torque axis into a gear ratio
```java
// if -ive, is torque
// if +ive, is boost
// if 0, is normal
final min_ratio = ...
final max_ratio = ...
final norm_ratio = ...
// is there a math function that can take a 0-1 value and scale it between min, max, normal
```
-> how do I implement this
SwerveModuleState consumer with a target gear ratio
-> Overload/one with default parameter for target gear ratio (default gear ratio) for normal driving/autonomous
-> How do I calculate the gear ratio?
    -> Do I want to? Is that actually useful? yes I think
    -> Rotation we do not care about
    -> Otherwise it's just a speed
    -> In teleop we adjust the max speed depending on gear ratio
        -> So, technically boost doesn't actually 'add' speed in this implementation, it just *allows* for more speed (although since speed = joystick * max_speed it will effectively increase speed).
    -> In teleop we scale the gear ratio depending on the torque and boost controls (what if we use them at the same time with the current implementation? Let's not try)
        -> Future implementation should add both input: `scale = (-torque) + boost` or `scale = boost - torque` either way they are equivalent (first is potentially more clear)
    -> Once `SwerveModuleKinematics.fromChassisSpeeds()` processes the user inputs (`ChassisSpeeds`), send them to this module with the specified gear ratio

-> The fun part: How to we calculate the speed to drive each motor based on the gear ratio (and potentially which motor is being used as a drive motor -> default to pulley motor if this turns out to be too difficult)
```java
// ideas:
// this example ignores the rotation component of a swervemodulestate
// -> can we increase PID slightly while rotating? that gain in performance is probably insignificant
public void setModuleState(SwerveModuleState desiredState, double gearRatio) {
    // limit gearRatio to Constants...min_ratio, Constants...max_ratio and display
    // limit speed to calculateMaxSpeed(gearRatio) and display
    // we have: speed, gear ratio
    // we need: planetaryVoltage, pulleyVoltage
    // we can get with: PIDF controllers
    // PIDF controller needs velocity of each motor -> how do we calculate this
    // 
    // planetary motor:
    // calculate PID
    // calculate FF
    // ^^^ use SparkMax PID? -> problem is doesn't allow for PID to be dynamically adjusted for more gear ratio
    // pulley motor:
    // calculate PID
    // calculate FF
    // ^^^ use SparkMax PID?
}
```

Teleop:
* Boost on right trigger
    * Input (0 - 1)
* Torque on left trigger
    * Input (0 - 1)
* Rotation velocity on right stick
    * Input (-1 - 1)
* Translation velocity on left stick
    * Input (x: -1 - 1)
    * Input (y: -1 - 1)

* Speeds are normalized to max speed:
    * Need a function that outputs the maximum speed given a gear ratio (this swerve can go to any)

* Torque needs to be handled specially (affects max speed)
* Boost can be handled via max speed

Auto:
* SwerveModuleState
    * Input (velocity: m/s)
    * input (rotation: Rotation2d)

Generic: