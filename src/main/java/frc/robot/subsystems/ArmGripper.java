// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
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
import frc.robot.Constants.Arm.Limits;
import frc.robot.Constants.Arm.Pneumatics;
import frc.robot.Constants.Arm.Ratios;
import frc.robot.Constants.Arm.SoftStop;

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
  private final DutyCycleEncoder m_lowerEncoderAbsolute = new DutyCycleEncoder(DIO.ARM_LOWER_ENCODER);
  private final DutyCycleEncoder m_upperEncoderAbsolute = new DutyCycleEncoder(DIO.ARM_UPPER_ENCODER);

  private final RelativeEncoder m_lowerEncoderRelative = m_lowerMotor.getEncoder();
  private final RelativeEncoder m_upperEncoderRelative = m_upperMotor.getEncoder();

  private final RelativeEncoder m_extensionEncoder = m_extensionMotor.getEncoder();
  // TODO: add other (relative) encoders here

  private final SparkMaxPIDController m_lowerPID = m_lowerMotor.getPIDController();
  private final SparkMaxPIDController m_upperPID = m_upperMotor.getPIDController();
  private final SparkMaxPIDController m_extensionPID = m_extensionMotor.getPIDController();

  XboxController m_operatorController;

  /** Creates a new ArmGripper. */
  public ArmGripper(XboxController operatorController) {
    m_compressor.enableDigital();

    m_lowerMotor.restoreFactoryDefaults();
    m_upperMotor.restoreFactoryDefaults();
    m_extensionMotor.restoreFactoryDefaults();
    configureEncoders();
    configureMotors();
    configurePIDs();

    m_operatorController = operatorController;
  }

  @Override
  public void periodic() {
    // TODO: Modify these as necessary.
    SmartDashboard.putNumber("Lower Arm Position (0-1)", m_lowerEncoderAbsolute.getAbsolutePosition());
    SmartDashboard.putNumber("Upper Arm Position (0-1)", m_upperEncoderAbsolute.getAbsolutePosition());
    // angles
    SmartDashboard.putNumber("Lower Arm Angle (Deg)", getLowerAngleAbsolute()); // positive away from robot
    SmartDashboard.putNumber("Lower Arm NEO Encoder Position", getLowerArmAngleRelative());
    // That does not work
    // SmartDashboard.putNumber("Upper Arm Relative Angle (Deg)", m_upperEncoderAbsolute.getDistance());
    SmartDashboard.putNumber("Upper Arm Angle (Deg)", getUpperArmAngleAbsolute()); // positive away from robot
    SmartDashboard.putNumber("Upper Arm NEO Encoder Position", getUpperArmAngleRelative());
    // lengths
    SmartDashboard.putNumber("Lower Arm Length (m)", getLowerArmLength());
    SmartDashboard.putNumber("Upper Arm Base Length (m)", getUpperArmBaseLength());
    SmartDashboard.putNumber("Upper Arm Total Length (m)", getUpperArmTotalLength());
    SmartDashboard.putNumber("Arm Extension Distance (m)", getExtensionDistance());
    // check solenoid status
    if (m_gripperSolenoid.isFwdSolenoidDisabled()) {
      System.out.print("OPEN SOLENOID DISABLED: CHECK FOR SHORTED/DISCONNECTED WIRES");
    }
    if (m_gripperSolenoid.isRevSolenoidDisabled()) {
      System.out.print("CLOSE SOLENOID DISABLED: CHECK FOR SHORTED/DISCONNECTED WIRES");
    }
  }

  private void configureEncoders() {
    // TODO: move to constants
    m_lowerEncoderRelative.setPositionConversionFactor(1.8);
    m_upperEncoderRelative.setPositionConversionFactor(1.8);

    // TODO: configure as necessary
    m_lowerEncoderAbsolute.setDistancePerRotation(Encoder.LOWER_DISTANCE_PER_ROTATION); // TODO: should not be 0
    m_upperEncoderAbsolute.setDistancePerRotation(Encoder.UPPER_DISTANCE_PER_ROTATION);
    m_extensionEncoder.setPositionConversionFactor(Encoder.EXTENSION_POSITION_CONVERSION);

    // Copy absolute position to NEO encoders
    m_lowerEncoderRelative.setPosition(getLowerAngleAbsolute());
    m_upperEncoderRelative.setPosition(getUpperArmAngleAbsolute());
  }

  private void configureMotors() {
    m_lowerMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
    m_lowerMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
    m_lowerMotor.setSoftLimit(SoftLimitDirection.kForward, SoftStop.LOWER_FORWARD); // degrees
    m_lowerMotor.setSoftLimit(SoftLimitDirection.kReverse, SoftStop.LOWER_REVERSE); // degrees
    m_lowerMotor.setIdleMode(IdleMode.kBrake);
    m_lowerMotor.setSmartCurrentLimit(Limits.LOWER_ARM_CURRENT);
    m_lowerMotor.setInverted(IsInverted.LOWER_MOTOR);

    m_upperMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
    m_upperMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
    m_upperMotor.setSoftLimit(SoftLimitDirection.kForward, SoftStop.UPPER_FORWARD); // degrees
    m_upperMotor.setSoftLimit(SoftLimitDirection.kReverse, SoftStop.UPPER_REVERSE); // degrees
    m_upperMotor.setIdleMode(IdleMode.kBrake);
    m_upperMotor.setSmartCurrentLimit(Limits.UPPER_ARM_CURRENT);
    m_upperMotor.setInverted(IsInverted.UPPER_MOTOR);

    m_extensionMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
    m_extensionMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
    m_extensionMotor.setSoftLimit(SoftLimitDirection.kForward, SoftStop.EXTENSION_FORWARD); // metres
    m_extensionMotor.setSoftLimit(SoftLimitDirection.kReverse, SoftStop.EXTENSION_REVERSE); // metres
    m_extensionMotor.setIdleMode(IdleMode.kBrake);
    m_extensionMotor.setSmartCurrentLimit(Limits.EXTENSION_CURRENT);
    m_extensionMotor.setInverted(IsInverted.EXTENSION_MOTOR);
  }

  private void configurePIDs() {
    m_lowerPID.setP(SmartDashboard.getNumber("Upper Arm P", 0.00007));
    m_lowerPID.setI(SmartDashboard.getNumber("Upper Arm I", 0.00000000005));
    m_lowerPID.setD(SmartDashboard.getNumber("Upper Arm D", 0.00000003));
    m_lowerPID.setIZone(0.5);
    m_lowerPID.setFF(0.00015);
    m_lowerPID.setOutputRange(-1.0, 1.0);
    m_lowerPID.setSmartMotionMaxVelocity(3000, 0);
    m_lowerPID.setSmartMotionMaxAccel(15000, 0);

    m_upperPID.setP(SmartDashboard.getNumber("Lower Arm P", 0.00007));
    m_upperPID.setI(SmartDashboard.getNumber("Lower Arm I", 0.00000000005));
    m_upperPID.setD(SmartDashboard.getNumber("Lower Arm D", 0.00000003));
    m_upperPID.setIZone(0.5);
    m_upperPID.setFF(0.00015);
    m_upperPID.setOutputRange(-1.0, 1.0);
    m_upperPID.setSmartMotionMaxVelocity(3000, 0);
    m_upperPID.setSmartMotionMaxAccel(15000, 0);

    m_extensionPID.setP(SmartDashboard.getNumber("Extension P", 0.00005));
    m_extensionPID.setI(SmartDashboard.getNumber("Extension I", 0.000000001));
    m_extensionPID.setD(SmartDashboard.getNumber("Extension D", 0.0000003));
    m_extensionPID.setIZone(0.5);
    m_extensionPID.setFF(0.0005);
    m_extensionPID.setOutputRange(-1.0, 1.0);
    m_extensionPID.setSmartMotionMaxVelocity(5000, 0);
    m_extensionPID.setSmartMotionMaxAccel(15000, 0);
  }

  /**
   * Returns the amount the arm extension has extended.
   * @return The amount the arm extension has extended in metres.
   */
  public double getExtensionDistance() {
    return m_extensionEncoder.getPosition();
  }

  /**
   * Returns the angle of the lower arm relative to the front of the robot using the absolute encoder.
   * WARNING: THIS MAY NOT ALWAYS BE ACCURATE! THIS FUNCTION SHOULD ONLY BE USED FOR OFFSET SETTING PURPOSES ONLY
   * @return The robot-relative angle of the lower arm in degrees.
   */
  private double getLowerAngleAbsolute() {
    // plus 90 because the offset was measured at 90.0 degrees
    return (((m_lowerEncoderAbsolute.getAbsolutePosition() - Encoder.LOWER_POSITION_OFFSET) * Ratios.LOWER_ARM_CHAIN) * 360.0) + 90.0;
  }

   /**
   * Returns the angle of the lower arm relative to the front of the robot using the absolute encoder.
   * @return The robot-relative angle of the lower arm in degrees.
   */
  public double getLowerArmAngleRelative(){
    return m_lowerEncoderRelative.getPosition();
  }

  /**
   * Returns the length of the lower arm.
   * @return The length of the lower arm in metres.
   */
  public double getLowerArmLength() {
    return Arm.LOWER_ARM_LENGTH;
  }

  /**
   * Returns the angle of the upper arm relative to the front of the robot using the absolute encoder.
   * WARNING: THIS MAY NOT ALWAYS BE ACCURATE! THIS FUNCTION SHOULD ONLY BE USED FOR OFFSET SETTING PURPOSES ONLY
   * @return The robot-relative angle of the upper arm in degrees.
   */
  private double getUpperArmAngleAbsolute() {
    /* The upper arm angle is relative to the lower arm. To calculate the
     * robot-relative angle of the upper arm, subtract the upper arm relative
     * angle from the lower arm angle.
     * TODO: show this in a markdown file
     * TODO: clean up encoder calculations and encoder calculation functions
     */
    return ((m_upperEncoderAbsolute.getAbsolutePosition()-Encoder.UPPER_POSITION_OFFSET)*Ratios.UPPER_ARM_CHAIN*360)+90;
  }

   /**
   * Returns the angle of the upper arm relative to the front of the robot using the absolute encoder.
   * @return The robot-relative angle of the upper arm in degrees.
   */
  public double getUpperArmAngleRelative(){
    return m_upperEncoderRelative.getPosition();
  }

  /**
   * Returns the length of the upper arm without the extension.
   * @return The length of the upper arm excluding the extension in metres.
   */
  public double getUpperArmBaseLength() {
    return Arm.UPPER_ARM_BASE_LENGTH;
  }

  /**
   * Returns the length of the upper arm + the current extension length.
   * @return The length of the upper arm including the extension in metres.
   */
  public double getUpperArmTotalLength() {
    return getExtensionDistance() + getUpperArmBaseLength();
  }

  /**
   * Instruct all motor PID controllers to hold their current position.
   */
  public void holdPosition() {
    m_lowerPID.setReference(getLowerArmAngleRelative(), ControlType.kSmartMotion);
    m_upperPID.setReference(getUpperArmAngleRelative(), ControlType.kSmartMotion);
    m_extensionPID.setReference(getExtensionDistance(), ControlType.kSmartMotion);
  }

  /** 
   * Control the arm position using the operator controller.
   * Does not control the gripper solenoids.
   */
  public void manualControl() {
    // set speeds of arm motors
    m_lowerMotor.set(MathUtil.applyDeadband(
        m_operatorController.getLeftY(), Xbox.JOYSTICK_DEADBAND));
        // not inverted so that lower arm moves in same direction as joystick
    m_upperMotor.set(MathUtil.applyDeadband(
        -m_operatorController.getRightY(), Xbox.JOYSTICK_DEADBAND));

    // set extension motor
    if (m_operatorController.getPOV() == 0) m_extensionMotor.set(Arm.MANUAL_EXTENSION_SPEED);
    else if (m_operatorController.getPOV() == 180) m_extensionMotor.set(-Arm.MANUAL_EXTENSION_SPEED);
    else m_extensionMotor.disable();
  }

  public void openGripper() {
    m_gripperSolenoid.set(kForward);
  }

  public void closeGripper() {
    m_gripperSolenoid.set(kReverse);
  }

  /** Set the lower arm angle relative to the front of the robot in degrees. */
  public void setLowerTargetAngle(double angle) {
    m_lowerPID.setReference(angle, ControlType.kSmartMotion);
  }

  /** Set the upper arm angle relative to the front of the robot in degrees. */
  public void setUpperTargetAngle(double angle) {
    m_upperPID.setReference(angle, ControlType.kSmartMotion);
  }

  /** Set the amount to extend the upper arm in metres. */
  public void setExtensionTargetLength(double length) {
    m_extensionPID.setReference(length, ControlType.kSmartMotion);
  }

  public void stopAllMotors() {
    m_lowerMotor.stopMotor();
    m_upperMotor.stopMotor();
    m_extensionMotor.stopMotor();
  }
}
