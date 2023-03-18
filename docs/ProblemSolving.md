# Issues and Solutions

## Some Spark Max functionality does not work properly
### **Description:**
* You are setting a property of a Spark Max, and it appears to ignore the property change.
* The Driver Station shows an error along the lines of:
```
5:37:27.870 PM
ERROR  8  [CAN SPARK MAX] IDs: 6, Invalid parameter id 151   
```

### **Cause:**
* You are attempting to set a property that was added in a newer version of the Spark Max's firmware. The Spark Max does not know how to perform your property change, or does not support your property.

### **Solution:**
* Plug your computer the USB-C port of the Spark Max that is not following property changes.
* Follow [these instructions](https://docs.revrobotics.com/sparkmax/rev-hardware-client/getting-started-with-the-rev-hardware-client/updating-device-firmware) to update its firmware.
* Restart or redeploy your robot code.

## Swerve Drivetrain Does Not Behave Holonomically
### **Description:**
* Attempting to decouple rotation from translation did not work as expected.
* Autonomous did not follow paths correctly (any rotation correction interfered with the robot's X and Y position).
* We could not spin while moving.

### **Cause:**
* We inverted the `Rotation2d` component of the `SwerveModuleState`, which never inverted the encoder values. This means that the module rotation motors spin in the opposite direction than what they report.
* After logging the auto path, we found out the motor setpoint and the encoder reading were opposite; since we did not invert the encoder, the robot thought the module was moving in the opposite direction that it really was.
* This was not apparent in Teleoperated mode, as the robot moves in the correct direction (the reported position is bad).
* Since auto cannot see the robot, autonomous assumes the robot is going in a completely incorrect direction, and tries to correct for it, causing issues.

### **Solution:**
* Consider logging the setpoint (desired position) of your drive and rotation wheels. This can help diagnose similar problems in the future.
* Instead of inverting the `Rotation2d` component of a `SwerveModuleState`, invert the motor via `CANSparkMax.setInverted()`.

## Swerve modules spin improperly
### **Description:**
* Swerve module rotates 180 degrees in the opposite direction after spinning >= 180 degrees in a direction
* Occurs after driving the robot in a circle

### **Cause:**
* Continous input is not configured for the rotation PID controller. 
* 180 degrees and -180 degrees are treated as equivalent points by `SwerveModuleState.optimize()`, so if continous input is disabled or not configured to 180 degrees/-180 degrees (or the appropriate unit for the PID controller), the PID will not see these as equivalent points. For example, if the module is at 170 degrees, and is told to increase its position by 20 (an angle of 190 degrees), then `SwerveModuleState.optimize()` will tell the module to move to -170 degrees (equivalent to 190 degrees). Since the PID controller does not know it can rotate past 180 degrees to get to -180, it will attempt to spin in the opposite direction to get to -180.

### **Solution:**
* Configure continous input for the rotation PID controller.
* Minimum: -180 degrees or -Pi radians
* Maximum: 180 degrees or Pi radians
* With WPILib's `PIDController`:
```java
pidController.enableContinousInput(-Math.PI, Math.PI);
```
* With `SparkMaxPIDController`:
```java
// you must set a conversion factor to convert to your desired unit first (this is necessary whenever using the Spark Max's PID with a specific unit)
// if you are using the PID to control position:
// motorController.getEncoder().setPositionConversionFactor(your-calculated-factor);
// if you are using the PID to control velocity
// motorController.getEncoder().setVelocityConversionFactor(your-calculated-factor);
pidController.setPositionPIDWrappingEnabled(true);
pidController.setPositionPIDWrappingMinInput(-Math.PI);
pidController.setPositionPIDWrappingMaxInput(Math.PI);
// update your Spark Max firmware if this appears to do nothing
```

## Inverted X and Y while driving and improper rotation with swerve drive X, Y directions are inverted while driving and wheels face each other in X pattern
### **Description:**
* X and Y directions are inverted while driving (e.g. robot moves left when moving joystick right)
> Above issue may not be present if controller input was inverted to account for this.
* When rotating the robot in place, wheels form an X pattern instead of rotating to form a circle.

### **Cause:**

### **Solution:**
**ROTATION BUSTED (all go in towards each other for some reason)**
**POSITIONING IS CORRECT -> MATCHES WPILIB SAMPLE & EXPECTED VALUES ASSUMING ROBOT COORDINATE SYSTEM**
// INVERT THE XBOX STICKS PLEASE *test which directions are wrong* -> its left/right I fixed it (is rotation correct)

Angle setpoints:
```
POS   NEG

NEG   POS

All @0.78 rad (Pi/4) -> 45 degrees
```
- This is completely correct :D
- Except it isn't.
- **Actually Reli might be right. --> he was not**

SET IS COUNTERCLOCKWISE POS
```
45 POS    45 NEG
RIGHT 45  LEFT 45
/              \

45 NEG    45 POS
LEFT 45   RIGHT 45
\              /
```

When motor is using clockwise pos:
```
45 POS    45 NEG
LEFT 45   RIGHT 45
\              /

45 NEG    45 POS
RIGHT 45  LEFT 45
/              \
```

HOW DO I CONVERT FROM COUNTERCLOCKWISE POS->CLOCKWISE POS
you invert it, :p

Yeah, INVERT THE MOTOR, not the SwerveModuleState...
If you invert the SwerveModuleState, you need to invert the encoder (such as by putting a - in front of the unit conversion).
> A much easier and less bug-prone way to do it is to use your motor controller's invert functionality, such as `CANSparkMax.setInverted(true)`.

## NavX Crashes Code with `isRaspian()` error:
**THIS HAS NOW BEEN FIXED. DO NOT FOLLOW THIS PROCEDURE.**

### **Cause:**
* NavX Vendor Library not yet updated for 2023.
* It tries to make a call to `isRaspbian()` which has been renamed to `isLinux()`.

### **Solution:**
* Uninstall NavX Vendor Library:
* Command Palette: Manage Vendor Libraries -> Manage current libraries -> Check NavX library, press OK to uninstall
* Install unofficial NavX Library:
* Command Palette: Manage Vendor Libraries -> Install new libraries (online) -> paste `https://raw.githubusercontent.com/rzblue/navx-frc/maven/navx_frc.json` and press Enter/Return

## Issues with FRC PathPlanner
**See `repos/docs/FRCPathPlanner.md` for issue descriptions/solutions.**

## ERROR -1154 HAL- CAN Recieve has Timed Out ... ArmGripper
### **Description:**
* Loss of gripper control
* Rev Pneumatics Hub loses power
* Blinking light on PDH output supplying pneumatics hub

### **Cause:**
* Too much power drawn through PDH fuse
  * 15A fuse can be blown by compressor at peak power or a shorted cable.

### **Solution:**
* Replace fuse with one of a greater rating.
* If this occurs twice, check for shorted wires.

## Robot refuses to enable through driver station but is connected
### **Description:**
* Robot is connected properly
* Pressing enable does nothing

### **Cause:**
* Undetermined.

### **Solution:**
* Close and reopen driver station.
* Rebooting the robot/disconnecting and reconnecting is not necessary.
