// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm_positions;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.arm.ArmPaths;
import frc.robot.commands.arm.MoveArmToPosition;
import frc.robot.subsystems.Arm;

public class MoveArmToPickup extends SequentialCommandGroup {
  /** Creates a new PickupThenExtend. */
  public MoveArmToPickup(Arm arm, boolean isReverse) {
    addCommands(
        new MoveArmToPosition(arm, ArmPaths.longThrowPickup, isReverse),
        new MoveArmToPosition(arm, ArmPaths.extendToPickup, isReverse)
    );
  }
}
