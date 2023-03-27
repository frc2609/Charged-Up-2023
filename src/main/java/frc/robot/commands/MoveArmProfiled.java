// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.Arm.Tolerances;
import frc.robot.subsystems.ArmGripper;

public class MoveArmProfiled extends CommandBase {
  double[] lowerSetpoint = {0};
  double[] upperSetpoint = {0};
  double[] extensionSetpoint = {0};
  double UpperError, UpperArm_P;
  double LowerError, LowerArm_P;
  ArmGripper m_armGripper;
  boolean holdLower, holdUpper, holdSlider;
  double prevLoop;
  int i = 0;

  /** Moves arm to given setpoint. Finishes once within tolerance */
  public MoveArmProfiled(double[] lowerSetpoint, double[] upperSetpoint, double[] extensionSetpoint, ArmGripper armGripper) {
    this.lowerSetpoint = lowerSetpoint;
    this.upperSetpoint = upperSetpoint;
    this.extensionSetpoint = extensionSetpoint;
    m_armGripper = armGripper;
    addRequirements(armGripper);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    prevLoop = Timer.getFPGATimestamp();
    m_armGripper.setLowerTargetAngle(lowerSetpoint[0]);
    m_armGripper.setUpperTargetAngle(upperSetpoint[0]);
    m_armGripper.setExtensionTargetLength(extensionSetpoint[0]);
    i++;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    double UPPER_SET, LOWER_SET, EXTENSION_SET = 0;
    double dt = Timer.getFPGATimestamp()-prevLoop;
    if(dt > 0.05){
      i += Math.floor(dt/0.05);
    }
    UPPER_SET = upperSetpoint[i];
    LOWER_SET = lowerSetpoint[i];
    EXTENSION_SET = extensionSetpoint[i];

    UpperError = Math.abs(m_armGripper.getUpperArmAngleRelative() - UPPER_SET);
    if (UpperError > 10.0) UpperArm_P = 0.00007 + (0.00001 * UpperError);
    else UpperArm_P = 0.00007;

    LowerError = Math.abs(m_armGripper.getLowerArmAngleRelative() - LOWER_SET);
    if (LowerError > 10.0) LowerArm_P = 0.00007 + (0.00001 * LowerError);
    else LowerArm_P = 0.00007;

    SmartDashboard.putNumber("lowerError", Math.abs(m_armGripper.getLowerArmAngleRelative()-LOWER_SET));
    SmartDashboard.putNumber("upperError", Math.abs(m_armGripper.getUpperArmAngleRelative()-UPPER_SET));
    SmartDashboard.putNumber("extensionError", Math.abs(m_armGripper.getExtensionDistance()-EXTENSION_SET));
    //SmartDashboard.putNumber("Lower Arm P", UpperArm_P);
    //SmartDashboard.putNumber("Upper Arm P", LowerArm_P);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    boolean isLowerInTolerance = Math.abs(m_armGripper.getLowerArmAngleRelative()-lowerSetpoint[lowerSetpoint.length-1]) < Tolerances.LOWER_ANGLE; 
    boolean isUpperInTolerance = Math.abs(m_armGripper.getUpperArmAngleRelative()-upperSetpoint[upperSetpoint.length-1]) < Tolerances.UPPER_ANGLE; 
    boolean isExtensionInTolerance = Math.abs(m_armGripper.getExtensionDistance()-upperSetpoint[upperSetpoint.length-1]) < Tolerances.EXTENSION_LENGTH;
    return isLowerInTolerance && isUpperInTolerance && isExtensionInTolerance;
  }
}
