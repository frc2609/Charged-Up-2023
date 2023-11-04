// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autogripper;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.arm_positions.MoveArmToGroundPickup;
import frc.robot.commands.arm_positions.MoveArmToStow;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Gripper;

public class AutoGroundPickup extends SequentialCommandGroup {
  /** Creates a new AutoGroundPickup. */
  public AutoGroundPickup(Arm arm, Gripper gripper) {
    addCommands(
        new MoveArmToGroundPickup(arm),
        new AutoClose(gripper, 3),
        new MoveArmToStow(arm)
    );
  }
}
