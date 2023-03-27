// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.MoveArmToHigh;
// import frc.robot.commands.VisionAlign;
import frc.robot.subsystems.ArmGripper;
import frc.robot.subsystems.SwerveDrive;

public class ScoreConeHigh extends SequentialCommandGroup {
  /** Creates a new ScoreConeHigh. */
  public ScoreConeHigh(SwerveDrive drive, ArmGripper arm) {
    addCommands(
        new ParallelCommandGroup(
            // new VisionAlign(drive, null),
            new MoveArmToHigh(arm)
        ),
        Commands.waitSeconds(0.5),
        new InstantCommand(arm::openGripper, arm)
    );
  }
}
