// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AutoClose;
import frc.robot.commands.MoveArmToSetpoint;
import frc.robot.commands.MoveArmToStow;
import frc.robot.subsystems.ArmGripper;
import static frc.robot.Constants.Arm.Position.*;

public class AutoPickupCube extends SequentialCommandGroup {
  /** Creates a new AutoPickupCube. */
  public AutoPickupCube(ArmGripper gripper) {
    addCommands(
        new MoveArmToSetpoint(STOW_LOWER, STOW_UPPER, 0.045, isFinished(), isFinished(), isFinished(), gripper),
        new InstantCommand(gripper::openGripper),
        new AutoClose(gripper, 200, 7),
        new InstantCommand(gripper::closeGripper),
        Commands.waitSeconds(0.2),
        new MoveArmToStow(gripper)
    );
  }
}
