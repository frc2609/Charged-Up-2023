// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ArmGripper;

public class MoveArmToSetpoint extends CommandBase {
  double LOWER_SET;
  double UPPER_SET;
  double EXTENSION_SET;
  boolean holdLower, holdUpper, holdSlider;

  final ArmGripper m_armGripper;
  /** Moves arm to given setpoint. Finishes once within tolerance */
  public MoveArmToSetpoint(double lowerSetpoint, double upperSetpoint, double extensionSetpoint, boolean holdLower, boolean holdUpper, boolean holdSlider, ArmGripper armGripper) {
    this.holdLower = holdLower;
    this.holdUpper = holdUpper;
    this.holdSlider = holdSlider;
    LOWER_SET = lowerSetpoint;
    UPPER_SET = upperSetpoint;
    EXTENSION_SET = extensionSetpoint;
    m_armGripper = armGripper;
    addRequirements(armGripper);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(holdLower){
      LOWER_SET = (m_armGripper.getLowerArmAngleRelative());
    }
    if(holdUpper){
      UPPER_SET = (m_armGripper.getUpperArmAngleRelative());
    }
    if(holdSlider){
      EXTENSION_SET = (m_armGripper.getExtensionDistance());
    }
    m_armGripper.setLowerTargetAngle(LOWER_SET);
    m_armGripper.setUpperTargetAngle(UPPER_SET);
    m_armGripper.setExtensionTargetLength(EXTENSION_SET);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putNumber("lowerError", Math.abs(m_armGripper.getLowerArmAngleRelative()-LOWER_SET));
    SmartDashboard.putNumber("upperError", Math.abs(m_armGripper.getUpperArmAngleRelative()-UPPER_SET));
    SmartDashboard.putNumber("extensionError", Math.abs(m_armGripper.getExtensionDistance()-EXTENSION_SET));
  }

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
