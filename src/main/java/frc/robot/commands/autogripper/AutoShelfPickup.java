// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autogripper;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.arm.ManualArmControl;
import frc.robot.commands.arm_positions.MoveArmToPickup;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Gripper;

public class AutoShelfPickup extends SequentialCommandGroup {
  /** Creates a new AutoShelfPickup. */
  public AutoShelfPickup(Arm arm, Gripper gripper, CommandXboxController controller) {
    addCommands(
        new MoveArmToPickup(arm, false),
        new ParallelDeadlineGroup(
            new AutoClose(gripper, 3),
            new ManualArmControl(arm, controller)
        )
    );
  }
}
