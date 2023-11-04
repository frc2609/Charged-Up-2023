// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm_positions;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.arm.ArmPaths;
import frc.robot.commands.arm.ManualArmControl;
import frc.robot.commands.arm.MoveArmToPosition;
import frc.robot.subsystems.Arm;

public class MoveArmToHigh extends SequentialCommandGroup {
  /** Creates a new MoveArmToHigh. */
  public MoveArmToHigh(Arm arm) {
    addCommands(
        new MoveArmToPosition(arm, ArmPaths.stowToIntToHigh, false)
    );
  }
  
  /** Creates a MoveArmToHigh that runs the path in reverse when the supplied
   * trigger becomes true. */
  public MoveArmToHigh(Arm arm, Trigger reverseButton, CommandXboxController operatorController) {
    this(arm); // copy the forward path
    addCommands(
        // wait for reverse trigger
        new ParallelDeadlineGroup(
            Commands.waitUntil(reverseButton::getAsBoolean),
            new ManualArmControl(arm, operatorController)
        ),
        // follow path in reverse
        new MoveArmToPosition(arm, ArmPaths.stowToIntToHigh, true)
    );
  }
}
