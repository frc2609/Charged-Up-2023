# CHECK BEFORE MATCH

## Technician

### Arm
* Arm in correct position (not exiting frame perimeter)
    * If not: Power off and adjust arm position
* Extension is fully retracted
    * If not: Power off and adjust extension position, or adjust extension position and reset robot code
* Extension is not stuck on front guard
    * If not: Power off and adjust extension position
### Autonomous
* Arm preloaded with cone or cube
    * If not: Power off and preload cone or cube
* Robot aligned with correct node to score at
    * If not: Align to correct node according to preload
### Pneumatics
* Pneumatics high pressure (compressor) gauge is at 120 psi
    * If too high: Check pressure switch for faults
    * If too low:
        * Operator enable robot and see if compressor runs
        * If running: Check lines for air leaks
        * If not running: Check compressor power, replace if powered
* Pneumatics low pressure (gripper) gauge is at 60 psi
    * If too high: Check regulator is set correctly
    * If too low: Check pneumatics system for leaks
* Pneumatics Hub is not flashing any LEDs (indicating a short)
    * If flashing: Check wires at flashing LEDs for shorts
* Gripper solenoid manual override buttons are sticking out with notch vertical
    * This indicates manual override is OFF
    * If on: Turn manual override off (twist blue part with screwdriver until vertical)
### Power
* Power Distribution Hub is not flashing any LEDs (indicating a tripped breaker/blown fuse)
    * If flashing: Check wires for a short
        * If breaker: Power cycle robot
        * If fuse: Replace fuse
* Battery voltage >= 12.5V
    * If not: Replace battery
        * If battery was fully charged, mark battery as bad
### Swerve
* Swerve module arrow aligned with cutout on frame
    * If not: Align and power cycle or reset robot code

## Driver/Operator
### Arm
* Arm absolute angle is correct
    * If not: Technician check absolute encoder plugged in, operator reset robot code
* Arm relative (NEO) angle is correct and within +- 2 degrees of absolute
    * If not: Technician check absolute encoder plugged in, operator reset robot code
### Auto
* Correct autonomous path is selected
    * If not: Select correct autonomous path in SmartDashboard or Shuffleboard
### Dashboard
* NetworkTables is updating
    * If not:
        * Check robot connection in Driver Station
        * Restart SmartDashboard/Shuffleboard
* Shuffleboard boolean buttons are not greyed out
    * If they are:
        * Close and reopen Shuffleboard
### Driver Station
* FMS connected
    * If not: Operator disable and reenable ethernet adapter in Device Manager
* Robot connected
    * If not: Operator check ethernet cable connection, if good operator contact field admin
* Robot has robot code
    * If not, proceed until fixed: Wait 20s, reset roboRIO, close and reopen Driver Station
* Controllers connected to driver station
    * If not: Operator reset driver station
* Driver and Operator controllers are correctly assigned
    * If not: Operator swap controllers in driver station
### Sensors
* NavX connected (No NavX/Swerve Drive Odometry errors in Riolog)
    * If not: Technician check NavX for power.
        * If powered, replace NavX, technician power cycle robot
        * If not powered, check NavX and MXP for shorts, technician power cycle robot
* Limelight connected and feed visible
    * If not: Technician check Limelight power
        * If powered: Technician check Limelight ethernet calbe
        * If not powered: Technician check Limelight PDH slot for blinking light (indicating short/blown fuse)
### Swerve
* Swerve module angles are all 0 degrees
    * If not: Technician check module alignment, operator reset robot code
* Drive and rotation NEOs are all around the same temperature
    * If some motors are not:
        * Technician check motor resistance to movement
            * Can also smell motor (look for burnt smell)
        * Technician remove motor and check gearbox for resistance
            * Repair gearbox if movement is difficult
        * Replace motor if motor is unusually difficult to move

### Pneumatics
* No solenoid errors reported in Riolog
    * If errors reported: Technician check solenoid wires for shorts, then operator reset robot code
### Power
* Battery voltage >= 12.5V
    * If not: Technician replace battery
        * If battery was fully charged, mark battery as bad