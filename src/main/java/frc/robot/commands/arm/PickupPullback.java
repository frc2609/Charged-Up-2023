// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.MoveArmProfiled;
import frc.robot.commands.MoveArmToStow;
import frc.robot.subsystems.ArmGripper;

public class PickupPullback extends SequentialCommandGroup {
  /** Creates a new PickupPullback. */
  public PickupPullback(ArmGripper armGripper) {
    addCommands(
        new MoveArmProfiled(armGripper, "PickupPullback", false),
        new MoveArmToStow(armGripper)
    );
  }
}
