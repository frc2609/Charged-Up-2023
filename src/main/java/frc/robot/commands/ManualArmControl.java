// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.Arm.SoftStop;
// import frc.robot.Constants.Arm.Tolerances;
import frc.robot.Constants.Arm;
import frc.robot.Constants.Xbox;
import frc.robot.subsystems.ArmGripper;

/** Control arm setpoints using joysticks and triggers. */
public class ManualArmControl extends CommandBase {
  private final ArmGripper m_armGripper;
  private final Timer m_timer = new Timer();
  private final XboxController m_operatorController;
  private double m_lowerSetpoint = Double.NaN; // degrees
  private double m_upperSetpoint = Double.NaN; // degrees
  private double m_extensionSetpoint = Double.NaN; // metres

  /** Creates a new ManualArmControl. */
  public ManualArmControl(
      ArmGripper armGripper,
      XboxController operatorController
  ) {
    m_armGripper = armGripper;
    m_operatorController = operatorController;
    addRequirements(armGripper);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_lowerSetpoint = m_armGripper.getLowerAngleRelative();
    m_upperSetpoint = m_armGripper.getUpperAngleRelative();
    m_extensionSetpoint = m_armGripper.getExtensionDistance();
    m_timer.restart();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    final double delta = m_timer.get();
    final double lowerArmIn = MathUtil.applyDeadband(-m_operatorController.getRightY(), Xbox.JOYSTICK_DEADBAND);
    final double upperArmIn = MathUtil.applyDeadband(-m_operatorController.getLeftY(), Xbox.JOYSTICK_DEADBAND);
    // right trigger extends (pos), left trigger retracts (neg)
    // TODO: APPLY DEADBAND
    final double extensionIn = m_operatorController.getRightTriggerAxis() - m_operatorController.getLeftTriggerAxis();
    // calculate acceleration for each part of the arm
    m_lowerSetpoint += lowerArmIn * Arm.MANUAL_LOWER_ACCELERATION * delta;
    m_upperSetpoint += upperArmIn * Arm.MANUAL_UPPER_ACCELERATION * delta;
    m_extensionSetpoint += extensionIn * Arm.MANUAL_EXTENSION_ACCELERATION * delta;
    // clamp setpoints to allowable setpoint values
    m_lowerSetpoint = MathUtil.clamp(m_lowerSetpoint, SoftStop.LOWER_REVERSE, SoftStop.LOWER_FORWARD);
    m_upperSetpoint = MathUtil.clamp(m_upperSetpoint, SoftStop.UPPER_REVERSE, SoftStop.UPPER_FORWARD);
    m_extensionSetpoint = MathUtil.clamp(m_extensionSetpoint, SoftStop.EXTENSION_REVERSE, SoftStop.EXTENSION_FORWARD);
    // log to SmartDashboard
    SmartDashboard.putNumber("Manual/Lower Target Angle", m_lowerSetpoint);
    SmartDashboard.putNumber("Manual/Upper Target Angle", m_upperSetpoint);
    SmartDashboard.putNumber("Manual/Extension Target Distance", m_extensionSetpoint);
    // apply setpoints
    m_armGripper.setLowerTargetAngle(m_lowerSetpoint);
    m_armGripper.setUpperTargetAngle(m_upperSetpoint);
    m_armGripper.setExtensionTargetLength(m_extensionSetpoint);
    // reset the timer so it tracks the time it takes to reach this again
    m_timer.reset();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_armGripper.holdPosition();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
