# BeaverLogger.java

## Usage
### Logging Values
**In `logMP()`**
1. Add the source of information to `logMP()`'s parameter list
1. Add `data.append(Type.toString(source.data) + ',');` in the desired position in `logMP`.
> From top to bottom, each value will appear from left to right when read by a spreadsheet (csv) viewer.
> * `Type` should be the type of the data being logged. For example, `Double` when logging a `double` value (e.g. 0.5).
> * `source` should be a source of data that `logMP()` has access to.
> * `data` should be the data being logged.

**In `saveTitles()`**
1. In the **same position as `logMP()`**, add `titles.append("value_name,");` to set the title column for the value. Note the `,` after the title.

### Setup
1. Add `BeaverLogger.getInstance().logMP(sources);` wherever you would like to log values. You need to call this in one place (all sources must be accessible).
> This function will log information on the same interval that it is called.

# PathLogger.java

## Usage
PathLogger acts as a data source for BeaverLogger.

### Setup
1. Add `private PathLogger m_pathLogger = new PathLogger();` to your drivetrain
1. Call `m_pathLogger.setSources(current_pose_supplier);` once in your drivetrain to register the current pose supplier function with m_pathLogger.
> `current_pose_supplier` should return a `Pose2d`.
1. Call the below function once so that data is sent to `PathLogger` instead of `SmartDashboard` when running a `PPSwerveControllerCommand`
```java
PPSwerveControllerCommand.setLoggingCallbacks(
  m_pathLogger::setActiveTrajectory,
  m_pathLogger::setTargetPose,
  m_pathLogger::setSetpoint,
  m_pathLogger::setError
);
```

### BeaverLogger
1. Add `m_pathLogger` to any call to `BeaverLogger::logMP(sources)`.

# WPILib Logging
[Documentation](https://docs.wpilib.org/en/stable/docs/software/telemetry/datalog.html)