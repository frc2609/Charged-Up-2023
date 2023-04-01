// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmGripper;
import frc.robot.subsystems.LED;

public class AutoClose extends CommandBase {
  private ArmGripper m_gripper;
  private int i; // name

  /** Creates a new AutoClose. */
  public AutoClose(ArmGripper gripper) {
    m_gripper = gripper;
    // should this require ArmGripper? probably not as it only uses gripper
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_gripper.isIntakeReadingValid() && m_gripper.getIntakeSensorDistance() < 105) {
      i++;
      LED.setGreen();
    } else {
      i = 0;
      LED.setIdle();
    }
    // TODO: move to a constant
    if (i >= 5) {
      m_gripper.closeGripper();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return i >= 5;
  }
}