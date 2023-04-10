// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

/** Add your docs here. */
public class ECVT {
  // motor 1: planetary motor
  // motor 2: pulley motor
  // planetary motor encoder
  // pulley motor encoder
  // (functions for simulated single encoder)
  public ECVT(
      // planetary motor ID
      // pulley motor ID
      // drive motor (enum either planetary or pulley motor)
  ) {
    // setup encoders
    // setup PIDs (variable?)
    // setup motors
  }
}

/*
 * How do I control this?
 * 
 * 
 */


/*
* **SWERVE DRIVE:**
* CONSTANTS and CALCULATIONS:
* MEMBERS:
* FUNCTIONALITY:
* Align ALL modules to 
* INTERFACE:
* LOGGING:
* Module rotation (not ECVT)
* DATA INPUT:
* * SwerveModuleState 
* * * speed in metres per second
* * * angular position of module (as Rotation2d)
* * Boost throttle (default 0)
* * Torque throttle (default 0)
* *
* *
* DATA OUTPUT:
* TELEOP:
* * Move in any direction
* * Take speed m/s
* * Backspin (what unit)
* * Torque % (0 - 1) // will affect speed how to deal with [NO]
*/