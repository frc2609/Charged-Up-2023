// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ArmGripper;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class OpenGripper extends InstantCommand {
  ArmGripper m_ArmGripper;
  /** Creates a new OpenGripper. */
  public OpenGripper(ArmGripper armGripper) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_ArmGripper = armGripper;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_ArmGripper.setGripper(m_ArmGripper.openValue);
  }
}
