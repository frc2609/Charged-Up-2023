// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

// import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import frc.robot.commands.arm.PickupPullback;
import frc.robot.commands.arm.PickupThenExtend;
import frc.robot.subsystems.ArmGripper;

public class PickupGrab extends SequentialCommandGroup {
  /** Creates a new PickupGrab. */
  public PickupGrab(ArmGripper gripper) {
    addCommands(
        new PickupThenExtend(gripper, false),
        new AutoClose(gripper)
        //Commands.waitSeconds(0.5),
        //new PickupPullback(gripper));
        //new MoveArmProfiled(gripper, "PickToStow"));
    );
  }
}
