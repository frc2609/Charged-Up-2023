// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.Arm;
import frc.robot.Constants.Arm.SoftStop;
import frc.robot.subsystems.ArmGripper;

/** Adjust the arm setpoints using the DPAD. */
public class ManualArmAdjustment extends CommandBase {
  // private double m_lowerSetpoint = 0;
  private double m_upperSetpoint = 0;
  private double m_extensionSetpoint = 0;
  private double m_lastPOVInput = -1;
  private final ArmGripper m_armGripper;
  private final XboxController m_operatorController;

  /** Creates a new ManualArmAdjustment. */
  public ManualArmAdjustment(
      ArmGripper armGripper,
      XboxController operatorController
  ) {
    m_armGripper = armGripper;
    m_operatorController = operatorController;
    addRequirements(m_armGripper);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // m_lowerSetpoint = m_armGripper.getLowerArmAngleRelative();
    m_upperSetpoint = m_armGripper.getUpperArmAngleRelative();
    m_extensionSetpoint = m_armGripper.getExtensionDistance();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    final double pov = m_operatorController.getPOV();
    if (pov == 0 && m_lastPOVInput != 0) {
      // dpad up
      m_upperSetpoint += Arm.MANUAL_UPPER_INCREMENT;
      m_lastPOVInput = 0;
    } else if (pov == 90 && m_lastPOVInput != 90) {
      // dpad right
      m_extensionSetpoint += Arm.MANUAL_EXTENSION_INCREMENT;
      m_lastPOVInput = 90;
    } else if (pov == 180 && m_lastPOVInput != 180) {
      // dpad down
      m_upperSetpoint -= Arm.MANUAL_UPPER_INCREMENT;
      m_lastPOVInput = 180;
    } else if (pov == 270 && m_lastPOVInput != 270) {
      // dpad left
      m_extensionSetpoint -= Arm.MANUAL_EXTENSION_INCREMENT;
      m_lastPOVInput = 270;
    }
    m_lastPOVInput = pov;
    // clamp setpoints to allowable setpoint values
    // m_lowerSetpoint = MathUtil.clamp(m_lowerSetpoint, SoftStop.LOWER_REVERSE, SoftStop.LOWER_FORWARD);
    m_upperSetpoint = MathUtil.clamp(m_upperSetpoint, SoftStop.UPPER_REVERSE, SoftStop.UPPER_FORWARD);
    m_extensionSetpoint = MathUtil.clamp(m_extensionSetpoint, SoftStop.EXTENSION_REVERSE, SoftStop.EXTENSION_FORWARD);
    // lower setpoint is not adjustable (determined to be unnecessary)
    m_armGripper.setUpperTargetAngle(m_upperSetpoint);
    m_armGripper.setExtensionTargetLength(m_extensionSetpoint);
    // log to SmartDashboard
    // SmartDashboard.putNumber("Manual/Lower Target Angle", m_lowerSetpoint);
    SmartDashboard.putNumber("Manual/Upper Target Angle", m_upperSetpoint);
    SmartDashboard.putNumber("Manual/Extension Target Distance", m_extensionSetpoint);
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
