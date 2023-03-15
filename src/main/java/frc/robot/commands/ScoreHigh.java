// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmGripper;

// TODO: make generic command with customizable setpoints
public class ScoreHigh extends CommandBase {
  final double LOWER_SET = 39.0;
  final double UPPER_SET = 165.0;
  final double EXTENSION_SET = 0.478;

  final ArmGripper m_armGripper;

  //high
  //elbow:165
  //shoulder:39
  //ext: 0.478

  /** Creates a new ScoreHigh. */
  public ScoreHigh(ArmGripper armGripper) {
    m_armGripper = armGripper;
    addRequirements(armGripper);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
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
  } // TODO: open claws

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false; // TODO: check if at setpoint
  }
}
