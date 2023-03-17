// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ArmGripper;
public class OpenGripper extends InstantCommand {
  private final ArmGripper m_armGripper;

  /** Creates a new OpenGripper. */
  public OpenGripper(ArmGripper armGripper) {
    m_armGripper = armGripper;
    // addRequirements(armGripper);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_armGripper.openGripper();
  }
}
