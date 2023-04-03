// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AutoClose;
import frc.robot.commands.MoveArmToSetpoint;
import frc.robot.commands.MoveArmToStow;
import frc.robot.subsystems.ArmGripper;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class GroundPickCube extends SequentialCommandGroup {
  /** Creates a new GroundPickCube. */
  public GroundPickCube(ArmGripper gripper) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new MoveArmToSetpoint(104.6, 19.0, 0.04, false, false, false, gripper), new AutoClose(gripper, 150), new MoveArmToStow(gripper));
  }
}
