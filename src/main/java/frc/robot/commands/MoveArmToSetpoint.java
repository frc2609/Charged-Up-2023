// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ArmGripper;

public class MoveArmToSetpoint extends CommandBase {
  final double LOWER_SET;
  final double UPPER_SET;
  final double EXTENSION_SET;

  final ArmGripper m_armGripper;
  /** Moves arm to given setpoint. Finishes once within tolerance */
  public MoveArmToSetpoint(double lowerSetpoint, double upperSetpoint, double extensionSetpoint, ArmGripper armGripper) {
    LOWER_SET = lowerSetpoint;
    UPPER_SET = upperSetpoint;
    EXTENSION_SET = extensionSetpoint;
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
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    boolean isLowerInTolerance = Math.abs(m_armGripper.getLowerArmAngleRelative()-LOWER_SET) < Constants.Tolerances.LOWER_ARM_ANGLE; 
    boolean isUpperInTolerance = Math.abs(m_armGripper.getUpperArmAngleRelative()-UPPER_SET) < Constants.Tolerances.UPPER_ARM_ANGLE; 
    boolean isExtensionInTolerance = Math.abs(m_armGripper.getExtensionDistance()-EXTENSION_SET) < Constants.Tolerances.EXTENSION;
    return isLowerInTolerance && isUpperInTolerance && isExtensionInTolerance;
  }
}
