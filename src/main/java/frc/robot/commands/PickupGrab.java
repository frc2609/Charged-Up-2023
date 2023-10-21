// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
// import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.gripper.PickupThenExtend;
import frc.robot.subsystems.Gripper;

public class PickupGrab extends SequentialCommandGroup {
  /** Creates a new PickupGrab. */
  public PickupGrab(Gripper gripper, XboxController operatorController) {
    addCommands(
        new PickupThenExtend(gripper, false),
        new ParallelDeadlineGroup(
            new AutoClose(gripper, 3),
            new ManualArmControl(gripper, operatorController)
        )
    );
    // Commands.waitSeconds(0.5),new PickupPullback(gripper)); //new MoveArmProfiled(gripper, "PickToStow"));
  }
}
