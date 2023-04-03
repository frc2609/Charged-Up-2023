// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.LED;
import frc.robot.commands.arm.PickupPullback;
import frc.robot.commands.arm.PickupThenExtend;
import frc.robot.subsystems.ArmGripper;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PickupGrab extends SequentialCommandGroup {
  /** Creates a new PickupGrab. */
  public PickupGrab(ArmGripper gripper, XboxController opStick) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new PickupThenExtend(gripper, false), new ParallelDeadlineGroup(new AutoClose(gripper, 125), new ManualArmControl(gripper, opStick)) ); //Commands.waitSeconds(0.5),new PickupPullback(gripper)); //new MoveArmProfiled(gripper, "PickToStow"));
  }
}
