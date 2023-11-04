// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.Autonomous.Deadline;
import frc.robot.commands.arm_positions.StowToMidToHigh;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.SwerveDrive;

public class ScoreConeHigh extends SequentialCommandGroup {
  /** Creates a new ScoreConeHigh. */
  public ScoreConeHigh(SwerveDrive drive, Arm arm, Gripper gripper) {
    addCommands(
        new ParallelCommandGroup(
            // cancel if movement takes too long
            new ParallelRaceGroup(
              Commands.waitSeconds(Deadline.MOVE_TO_HIGH),
              new StowToMidToHigh(arm)
            )
        ),
        // wait for the arm to settle
        Commands.waitSeconds(1),
        new InstantCommand(gripper::openGripper, gripper)
    );
  }
}
