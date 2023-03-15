// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmGripper;

public class ScoreMid extends CommandBase {
  final double LOWER_SET = 60.0;
  final double UPPER_SET = 148.0;
  final double EXTENSION_SET = 0.02;
  // mid
  // Elbow: 145
  // Shoulder: 56
  // ext: 0.0317

  final ArmGripper m_armGripper;

  /** Creates a new ScoreMid. */
  public ScoreMid(ArmGripper armGripper) {
    m_armGripper = armGripper;
    addRequirements(armGripper);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // TODO: function which sets all three (and will check all three setpoints)
    System.out.println("MID");
    m_armGripper.setLowerTargetAngle(LOWER_SET);
    m_armGripper.setUpperTargetAngle(UPPER_SET);
    m_armGripper.setExtensionTargetLength(EXTENSION_SET);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_armGripper.holdPosition();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // TODO: functions for checking if arm is at setpoint
    // return m_armGripper.
    return false;
  }
}
