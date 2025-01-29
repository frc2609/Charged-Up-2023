// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autogripper;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.arm_positions.MoveArmToLow;
import frc.robot.commands.arm_positions.MoveArmToStow;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Gripper;

public class AutoScoreLow extends SequentialCommandGroup {
  /** Creates a new AutoScoreLow. */
  public AutoScoreLow(Arm arm, Gripper gripper) {
    addCommands(
      // should actually make MoveArmToLow end correctly...
      new ParallelRaceGroup(new MoveArmToLow(arm), Commands.waitSeconds(0.25)),
      new InstantCommand(gripper::openGripper),
      // reversing MoveArmToLow will drag the game piece out of low
      // stow retracts extension first and won't show the piece out of the way
      new MoveArmToStow(arm)
    );
  }
}
