// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.MoveArmProfiled;
import frc.robot.subsystems.ArmGripper;

public class ShortThrowMid extends SequentialCommandGroup {
  /** Creates a new ShortThrowMid. */
  public ShortThrowMid(ArmGripper gripper) {
    addCommands(
        new MoveArmProfiled(gripper, "ShortThrowMidPrep", false),
        
        new ParallelRaceGroup(new MoveArmProfiled(gripper, "ReachMid", false), Commands.waitSeconds(2))
    );
  }
}
