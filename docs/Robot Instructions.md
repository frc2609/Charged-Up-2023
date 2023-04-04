# How to use the robot

## Pre-match setup
* **Don't** turn on the robot yet
* Place the robot in the correct position
* Align the swerve modules (see Swerve module alignment)
* Now you can turn the robot on

## Swerve module alignment
**THESE ALIGNMENT INSTRUCTIONS ARE FOR THE SWERVE PRACTICE BOT.**

**FOR ROAD RUNNER, ALIGN THE NOTCH ON EACH MODULE WITH THE NOTCH ON THE ALUMINUM PLATE.**

Before turning on the robot, you must line up all four swerve modules so that the black gear on each wheel faces to the left of the robot:

<img src="images/aligned.jpg" alt="Wheels aligned, view from front right of the robot." width="350"/>

Alternatively, align the wheels so that the gold on the top of each module faces to the left of the robot:

<img src="images/aligned_birds-eye-view.jpg" alt="Wheels aligned, view from top of the robot." width="350"/>

The battery is located at the rear of the robot. Front is the direction facing away from the battery.

<img src="images/directions_birds-eye-view.jpg" alt="Robot frame with arrow pointing forwards, away from the battery at the rear." width="350"/>

Misaligned wheels will make it impossible to drive the robot properly:

<img src="images/misaligned.jpg" alt="Facing towards rear right of robot frame showing wheels pointing towards each other." width="350"/>

You have to look at the gears to verify the wheels are aligned correctly. The wheels are all straight, however, the two left modules will spin backwards:

(Notice the gold gears on the left two modules are facing towards the right.)

<img src="images/misaligned_birds-eye-view.jpg" alt="Left modules misaligned, view from top of the robot" width="350"/>

To line up the wheels, lift up the corner of the robot and turn the wheel by hand. Once the module faces the correct direction, spin the small NEO on the module to make fine adjustments.

If the robot is already turned on, disable it and line up the wheels, then press "Reset Encoders" in Shuffleboard or reset the robot code (either through the driver station or by pressing the "Reset" button on the roboRIO).

## Using the robot
### Field Relative (Field Oriented) Drive
When the robot is driven with field-relative (or field oriented) drive turned off, horizontal and vertical movement is relative to the robot. For example, if the robot is facing to the right of the field, pushing the joystick forward will move the robot towards the right (where it is facing).

With field oriented drive turned on, horizontal and vertical movement is relative to the **field**. If the robot is facing in the same direction as before (the right), then telling it to move forward will move it away from you, while continuing to point to the right.

Rotation remains the same whether field oriented is on or off.

Note that field oriented drive does not need an actual field to function. When the robot is turned on, the robot assumes that it is in one of the left corners of the field, facing towards the field's centre.

If you drive the robot with no field, line the robot up in front of the driver station, and make sure the robot faces forward (away from the driver station). This will make it more intuitive to drive (moving left, right, forward, or back will move the robot in that direction relative to the driver station) during a demonstration or while testing the robot.

### Gyro Drift
The NavX IMU's built in gyroscope is used to keep track of the direction the robot is facing while in field-oriented mode. Like all gyros, the NavX's gyro drifts, although in a regular match it will not drift enough to be noticeable.

If you are using the robot for a demonstration, you may have it on long enough for it to be affected by gyro drift. To calibrate the gyro, align the robot so it faces forward and then press `Zero Yaw` on Smartdashboard or push the appropriate button on the driver's Xbox controller (see `Controls` below).

### Field Position Inaccuracy
To determine its position relative to the field, the robot relies on the total amount of rotations made by the drive motor of each swerve module. **If you picked up the robot and moved it to a different location, it *will not* know that its position changed!**

While the robot is being driven, wheels can slip and the robot can bump against objects on the field, or be pushed around by other robots. This can cause the reported position to differ from the actual position of the robot.

Since making contact with other robots during autonomous mode is prohibited (and you are hopefully not colliding with game objects or the field), the reported position of the robot will be accurate. **You can safely use the robot position in autonomous mode.**

**Going over the charge platform or the cable guard will cause the robot to lose its autonomous position.** In order to track its position correctly, the robot must get its position using the AprilTags on the field (or any other method not involving `SwerveDriveOdometry`'s dead reckoning).

In teleop mode, the robot can collide with parts of the field, and other robots, which will cause the reported position to differ from the actual position. **In teleop mode, you should *not* rely on the robot's reported position to be accurate!** This also means that the robot position in the Field2d widget in Shuffleboard may not correspond to its actual position.

## Controls
### Driver:
| Axis             | Function |
| ---------------- | -------- |
| Left Joystick X  | Drive left/right
| Left Joystick Y  | Drive forward/back
| Right Joystick X | Rotate left/right
| Right Joystick Y | NA
| Left Trigger     | NA
| Right Trigger    | Boost throttle

| Button       | Function |
| ------------ | -------- |
| Left Stick   | Request cone
| Right Stick  | Request cube
| Left Bumper  | Move to ground pickup
| Right Bumper | Move to station pickup
| Start        | Reset odometry yaw
| Select       | **DISABLED** (Hold) Lock charge position
| A            | Enable Boost
| B            | Autoalign to cone node
| X            | Move to stow position
| Y            | Confirm queued command

| DPAD  | Function |
| ----- | -------- |
| Up    | NA
| Down  | NA
| Left  | NA
| Right | NA

### Operator:
| Axis             | Function                  |
| ---------------- | ------------------------- |
| Left Joystick X  | NA                        |
| Left Joystick Y  | Manual: Move upper arm    |
| Right Joystick X | NA                        |
| Right Joystick Y | Manual: Move lower arm    |
| Left Trigger     | Manual: Retract extension |
| Right Trigger    | Manual: Extend extension  |

| Button       | Function |
| ------------ | -------- |
| Left Stick   | (Press) Request cone (Hold) Urgent request cone
| Right Stick  | (Press) Request cube (Hold) Urgent request cube
| Left Bumper  | Open gripper
| Right Bumper | Close gripper
| Start        | NA
| Select       | Reset arm encoders to absolute
| A            | Queue move to low
| B            | Queue move to mid
| X            | Move to stow position
| Y            | Queue move to high

* How to remember queue buttons:
    * Y is highest = high node
    * A is lowest = low node
    * B is in between = middle node

| DPAD  | Function |
| ----- | -------- |
| Up    | Manual Adjust: Increase upper setpoint
| Down  | Manual Adjust: Decrease upper setpoint
| Left  | Manual Adjust: Retract extension
| Right | Manual Adjust: Extend extension

* These controls are only enabled while `ManualArmAdjust` is being used (you must modify the robot code to use it). By default, the DPAD is not mapped to anything.

## Dashboard
This repository comes with a premade Shuffleboard layout. To use it, open Shuffleboard, open the File menu and press Load layout, then select `robot-shuffleboard-layout.json` in the root of this repository.

### Drivetrain Tab
Swerve Drive related data and buttons are available in the `Drivetrain` tab in Shuffleboard.

This tab contains information on each module's angle, velocity, and distance driven, as well as the robot's heading and field position.

The Field in the centre of the tab shows the robot's position in the field. Below it is the robot's heading, and buttons for setting up the drivetrain.
- `Zero Yaw` resets the robot's heading.
- `Reset Encoders` resets the reported position of the drive and rotation encoders. Line up the wheels and then press this button to reset their reported position.
- `Is Field Relative` toggles field-relative (field-oriented) drive. You can turn this on or off at any time (the robot will remember where it is even if this is turned off).