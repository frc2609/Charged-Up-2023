// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Gripper extends SubsystemBase {
  XboxController m_operatorController;

  /** Creates a new Gripper. */
  public Gripper(XboxController operatorController) {
    m_operatorController = operatorController;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
