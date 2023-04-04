// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ArmGripper;

public class StowMidToHigh extends SequentialCommandGroup {
  /** Creates a new StowMidToHigh. */
  public StowMidToHigh(ArmGripper gripper) {
    addCommands(
      new MoveArmProfiled(gripper, "LongThrowPickup", false),
      new MoveArmProfiled(gripper, "PickupToHigh", false)
    );
  }
}
