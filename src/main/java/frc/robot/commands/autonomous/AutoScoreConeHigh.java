// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.MoveArmToHigh;
import frc.robot.subsystems.ArmGripper;

public class AutoScoreConeHigh extends SequentialCommandGroup {
  /** Creates a new AutoScoreConeHigh. */
  public AutoScoreConeHigh(ArmGripper armGripper) {
    addCommands(
        new MoveArmToHigh(armGripper),
        new InstantCommand(armGripper::openGripper)
    );
  }
}
