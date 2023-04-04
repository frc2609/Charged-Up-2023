// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.Arm.Tolerances;
import frc.robot.subsystems.ArmGripper;

public class MoveArmToSetpoint extends CommandBase {
  private static final int MAX_LOOP_ITERATIONS = 3;
  private double m_lowerSetpoint, m_upperSetpoint, m_extensionSetpoint;
  private final boolean m_holdLower, m_holdUpper, m_holdSlider;
  private final ArmGripper m_armGripper;
  private int loopsAtSetpoint = 0;

  /** Moves arm to given setpoint. Finishes once within tolerance. */
  public MoveArmToSetpoint(
      double lowerSetpoint,
      double upperSetpoint,
      double extensionSetpoint,
      boolean holdLower,
      boolean holdUpper,
      boolean holdSlider,
      ArmGripper armGripper
  ) {
    m_holdLower = holdLower;
    m_holdUpper = holdUpper;
    m_holdSlider = holdSlider;
    m_lowerSetpoint = lowerSetpoint;
    m_upperSetpoint = upperSetpoint;
    m_extensionSetpoint = extensionSetpoint;
    m_armGripper = armGripper;
    addRequirements(armGripper);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    loopsAtSetpoint = 0;
    if (m_holdLower) {
      m_lowerSetpoint = m_armGripper.getLowerAngleRelative();
    }
    if (m_holdUpper) {
      m_upperSetpoint = m_armGripper.getUpperAngleRelative();
    }
    if (m_holdSlider) {
      m_extensionSetpoint = m_armGripper.getExtensionDistance();
    }
    m_armGripper.setLowerTargetAngle(m_lowerSetpoint);
    m_armGripper.setUpperTargetAngle(m_upperSetpoint);
    m_armGripper.setExtensionTargetLength(m_extensionSetpoint);
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
    if (isWithinTolerance()) loopsAtSetpoint++;
    else loopsAtSetpoint = 0; // reset if no longer at setpoint
    // wait until at setpoint for a certain amount of time
    return loopsAtSetpoint >= MAX_LOOP_ITERATIONS;
  }

  private boolean isWithinTolerance() {
    double lowerError = m_armGripper.getLowerAngleRelative() - m_lowerSetpoint;
    double upperError = m_armGripper.getUpperAngleRelative() - m_upperSetpoint;
    double extensionError = m_armGripper.getExtensionDistance() - m_extensionSetpoint;
    boolean isLowerInTolerance = Math.abs(lowerError) < Tolerances.LOWER_ANGLE; 
    boolean isUpperInTolerance = Math.abs(upperError) < Tolerances.UPPER_ANGLE;
    boolean isExtensionInTolerance = Math.abs(extensionError) < Tolerances.EXTENSION_LENGTH;
    if(m_holdLower){
      isLowerInTolerance = true;
    }
    if(m_holdUpper){
      isUpperInTolerance = true;
    }
    if(m_holdSlider){
      isExtensionInTolerance = true;
    }
    return isLowerInTolerance && isUpperInTolerance && isExtensionInTolerance;
  }
}
