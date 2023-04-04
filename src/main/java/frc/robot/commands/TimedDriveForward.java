// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import frc.robot.commands.autonomous.DriveForwardWhileSpinning;
import frc.robot.subsystems.SwerveDrive;

public class TimedDriveForward extends ParallelDeadlineGroup {
  /** Creates a new TimedDriveForward. */
  public TimedDriveForward(double timerDelay, SwerveDrive swerveDrive) {
    // Add the deadline command in the super() call. Add other commands using
    // addCommands().
    super(Commands.waitSeconds(timerDelay));
    addCommands(new DriveForwardWhileSpinning(swerveDrive, 2, Math.PI, 2));
  }
}
