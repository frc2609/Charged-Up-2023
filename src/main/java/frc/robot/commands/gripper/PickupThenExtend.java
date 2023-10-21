// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.gripper;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.MoveArmProfiled;
import frc.robot.subsystems.ArmGripper;

public class PickupThenExtend extends SequentialCommandGroup {
  /** Creates a new PickupThenExtend. */
  public PickupThenExtend(ArmGripper armGripper, boolean isReverse) {
    addCommands(
        new MoveArmProfiled(armGripper, "LongThrowPickup",isReverse),
        new MoveArmProfiled(armGripper, "ExtendToPickup",isReverse)
    );
  }
}
