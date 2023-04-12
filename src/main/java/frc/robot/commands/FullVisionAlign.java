// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.SwerveDrive;

public class FullVisionAlign extends SequentialCommandGroup {
  /** Creates a new FullVisionAlign. */
  public FullVisionAlign(SwerveDrive swerveDrive, XboxController driverController) {
    addCommands(
        new AlignToRotation(Rotation2d.fromDegrees(180), swerveDrive, driverController),
        new AlignToNode(swerveDrive) // should wait until rotation is aligned before going (it technically still does move to rotation but should ignore it since it's already lined up)
    );
  }
}
