// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.SwerveDrive;

public class SetOdometry extends InstantCommand {
  SwerveDrive drive;
  Pose2d pose;
  public SetOdometry(SwerveDrive drive, Pose2d pose) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drive = drive;
    this.pose = pose;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drive.resetPose(pose);
  }
}
