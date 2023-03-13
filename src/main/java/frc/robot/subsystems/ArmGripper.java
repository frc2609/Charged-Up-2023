// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.Arm;
import frc.robot.Constants.CANID;
import frc.robot.Constants.DIO;
import frc.robot.Constants.Xbox;
import frc.robot.Constants.Arm.Encoder;
import frc.robot.Constants.Arm.IsInverted;
import frc.robot.Constants.Arm.Pneumatics;

public class ArmGripper extends SubsystemBase {
  private final Compressor m_compressor =
      new Compressor(CANID.PNEUMATICS_HUB, PneumaticsModuleType.REVPH);

  private final DoubleSolenoid m_gripperSolenoid = new DoubleSolenoid(
      CANID.PNEUMATICS_HUB,
      PneumaticsModuleType.REVPH, 
      Pneumatics.OPEN_SOLENOID_ID, // fwd
      Pneumatics.CLOSE_SOLENOID_ID // rev
  );

  private final CANSparkMax m_lowerMotor = new CANSparkMax(CANID.LOWER_ARM_MOTOR, MotorType.kBrushless);
  private final CANSparkMax m_upperMotor = new CANSparkMax(CANID.UPPER_ARM_MOTOR, MotorType.kBrushless);
  private final CANSparkMax m_extensionMotor = new CANSparkMax(CANID.EXTENSION_MOTOR, MotorType.kBrushless);

  // Absolute encoder range is 0 to 1
  private final DutyCycleEncoder m_lowerEncoder = new DutyCycleEncoder(DIO.ARM_LOWER_ENCODER);
  private final DutyCycleEncoder m_upperEncoder = new DutyCycleEncoder(DIO.ARM_UPPER_ENCODER);

  private final RelativeEncoder m_extensionEncoder = m_extensionMotor.getEncoder();

  XboxController m_operatorController;

  /** Creates a new ArmGripper. */
  public ArmGripper(XboxController operatorController) {
    m_lowerMotor.setInverted(IsInverted.LOWER_MOTOR);
    m_upperMotor.setInverted(IsInverted.UPPER_MOTOR);
    m_extensionMotor.setInverted(IsInverted.EXTENSION_MOTOR);

    m_lowerMotor.setIdleMode(IdleMode.kBrake);
    m_upperMotor.setIdleMode(IdleMode.kBrake);
    m_extensionMotor.setIdleMode(IdleMode.kBrake);

    m_compressor.enableDigital();

    m_lowerEncoder.setPositionOffset(Encoder.LOWER_POSITION_OFFSET);
    m_upperEncoder.setPositionOffset(Encoder.UPPER_POSITION_OFFSET);

    // TODO: For absolute encoders, use this with getDistance(), not getAbsolutePosition()
    m_lowerEncoder.setDistancePerRotation(Encoder.LOWER_DISTANCE_PER_ROTATION);
    m_upperEncoder.setDistancePerRotation(Encoder.UPPER_DISTANCE_PER_ROTATION);
    m_extensionEncoder.setPositionConversionFactor(Encoder.EXTENSION_POSITION_CONVERSION);

    m_operatorController = operatorController;
  }

  @Override
  public void periodic() {
    // TODO: Modify these as necessary.
    SmartDashboard.putNumber("Lower Arm Position (0-1)", m_lowerEncoder.getAbsolutePosition());
    SmartDashboard.putNumber("Upper Arm Position (0-1)", m_upperEncoder.getAbsolutePosition());
    SmartDashboard.putNumber("Arm Extension Length (n/a)", m_extensionEncoder.getPosition());
    if (m_gripperSolenoid.isFwdSolenoidDisabled()) {
      System.out.print("OPEN SOLENOID DISABLED: CHECK FOR SHORTED/DISCONNECTED WIRES");
    }
    if (m_gripperSolenoid.isRevSolenoidDisabled()) {
      System.out.print("CLOSE SOLENOID DISABLED: CHECK FOR SHORTED/DISCONNECTED WIRES");
    }
  }

  /** Control the arm using the operator controller. */
  public void manualControl() {
    // set speeds of arm motors
    m_lowerMotor.set(MathUtil.applyDeadband(
      -m_operatorController.getLeftY(), Xbox.JOYSTICK_DEADBAND));
    m_upperMotor.set(MathUtil.applyDeadband(
      -m_operatorController.getRightY(), Xbox.JOYSTICK_DEADBAND));
    // set gripper solenoid
    if (m_operatorController.getAButtonPressed()) {
      m_gripperSolenoid.set(kForward);
    }
    if (m_operatorController.getBButtonPressed()) {
      m_gripperSolenoid.set(kReverse);
    }
    // set extension motor
    if (m_operatorController.getXButton()) {
      m_extensionMotor.set(Arm.MANUAL_EXTENSION_SPEED);
    } else if (m_operatorController.getYButton()) {
      m_extensionMotor.set(-Arm.MANUAL_EXTENSION_SPEED);
    } else {
      m_extensionMotor.disable();
    }
  }
}
