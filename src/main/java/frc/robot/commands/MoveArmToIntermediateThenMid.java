// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ArmGripper;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class MoveArmToIntermediateThenMid extends SequentialCommandGroup {
  /** Creates a new MoveArmToIntermediateThenMid. */
  public MoveArmToIntermediateThenMid(ArmGripper armGripper) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new MoveArmToSetpoint(70, armGripper.getUpperArmAngleRelative(), armGripper.getExtensionDistance(), armGripper), new TimerDelay(3), new ScoreMid(armGripper), new TimerDelay(0.2), new OpenGripper(armGripper));
  }
}
