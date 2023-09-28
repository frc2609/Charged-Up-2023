// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.ManualArmControl;
import frc.robot.commands.MoveArmProfiled;
import frc.robot.subsystems.ArmGripper;

public class StowMidToHigh extends SequentialCommandGroup {
  /** Creates a new StowMidToHigh. */
  public StowMidToHigh(ArmGripper gripper) {
    addCommands(
        new MoveArmProfiled(gripper, "LongThrowPickup", false),
        new ParallelRaceGroup(new MoveArmProfiled(gripper, "PickupToHigh", false),Commands.waitSeconds(2))
    );
  }
  
  /** Creates a new StowMidToHigh (add info here) */
  public StowMidToHigh(ArmGripper gripper, Trigger reverseButton, XboxController operatorController) {
    this(gripper);
    addCommands(
        new ParallelDeadlineGroup(
            Commands.waitUntil(reverseButton::getAsBoolean),
            new ManualArmControl(gripper, operatorController)
        ),
        new MoveArmProfiled(gripper, "PickupToHigh", true),
        Commands.waitSeconds(0.5),
        new MoveArmProfiled(gripper, "LongThrowPickup", true)
    );
  }
}
