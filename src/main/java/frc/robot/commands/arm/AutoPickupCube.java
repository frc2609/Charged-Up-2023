// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AutoClose;
import frc.robot.commands.MoveArmToStow;
import frc.robot.subsystems.ArmGripper;

public class AutoPickupCube extends SequentialCommandGroup {
  /** Creates a new AutoPickupCube. */
  public AutoPickupCube(ArmGripper gripper) {
    addCommands(
        new MoveArmToStow(gripper),
        new InstantCommand(gripper::closeGripper),
        new AutoClose(gripper, 196)
    );
  }
}
