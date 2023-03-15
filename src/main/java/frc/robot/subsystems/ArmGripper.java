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
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.Arm;
import frc.robot.Constants.CANID;
import frc.robot.Constants.DIO;
import frc.robot.Constants.Xbox;
import frc.robot.Constants.Arm.Encoder;
import frc.robot.Constants.Arm.IsInverted;
import frc.robot.Constants.Arm.Pneumatics;
import frc.robot.Constants.Arm.Ratios;

public class ArmGripper extends SubsystemBase {
  private final Compressor m_compressor =
      new Compressor(CANID.PNEUMATICS_HUB, PneumaticsModuleType.REVPH);
  
  public Value openValue = kReverse;
  public Value closeValue = kForward; // TODO: Double check these and move to constants

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
    m_lowerPID.setP(0.000025);//0.00005);
    m_lowerPID.setI(0.00001);//0.00000025);//0.000000001);
    m_lowerPID.setD(0.0);//0.005);//0.01);//0);//0.0000005);
    
    SmartDashboard.putNumber("Upper Arm P", 0.00025);
    SmartDashboard.putNumber("Upper Arm I", 0.00001);
    SmartDashboard.putNumber("Upper Arm D", 0.0);
    
    SmartDashboard.putNumber("Lower Arm P", 0.00025);
    SmartDashboard.putNumber("Lower Arm I", 0.00001);
    SmartDashboard.putNumber("Lower Arm D", 0.0);
    
    SmartDashboard.putNumber("Extension P", 0.00005);
    SmartDashboard.putNumber("Extension I", 0.000000001);
    SmartDashboard.putNumber("Extension D", 0.0000005);
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
    m_lowerMotor.getEncoder().setPositionConversionFactor(1.8);
    m_upperMotor.getEncoder().setPositionConversionFactor(1.8);

    // TODO: configure as necessary
    m_lowerEncoderAbsolute.setDistancePerRotation(Encoder.LOWER_DISTANCE_PER_ROTATION);
    m_upperEncoderAbsolute.setDistancePerRotation(Encoder.UPPER_DISTANCE_PER_ROTATION);
    m_extensionEncoder.setPositionConversionFactor(Encoder.EXTENSION_POSITION_CONVERSION);

  }

  private void configureMotors() {
    // m_lowerMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
    // m_lowerMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
    // m_lowerMotor.setSoftLimit(SoftLimitDirection.kForward, 356); // degrees
    // m_lowerMotor.setSoftLimit(SoftLimitDirection.kReverse, 0); // degrees
    m_lowerMotor.setIdleMode(IdleMode.kBrake);
    m_lowerMotor.setSmartCurrentLimit(40); // TODO: move to constants
    m_lowerMotor.setInverted(IsInverted.LOWER_MOTOR);

    // m_upperMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
    // m_upperMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
    // m_upperMotor.setSoftLimit(SoftLimitDirection.kForward, 356); // degrees
    // m_upperMotor.setSoftLimit(SoftLimitDirection.kReverse, 0); // degrees
    m_upperMotor.setIdleMode(IdleMode.kBrake);
    m_upperMotor.setSmartCurrentLimit(40);
    m_upperMotor.setInverted(IsInverted.UPPER_MOTOR);

    // m_extensionMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
    // m_extensionMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
    // m_extensionMotor.setSoftLimit(SoftLimitDirection.kForward, 356); // metres
    // m_extensionMotor.setSoftLimit(SoftLimitDirection.kReverse, 0); // metres
    m_extensionMotor.setIdleMode(IdleMode.kBrake);
    m_extensionMotor.setSmartCurrentLimit(40);
    m_extensionMotor.setInverted(IsInverted.EXTENSION_MOTOR);
  }

  private void configurePIDs() {
    m_lowerPID.setP(SmartDashboard.getNumber("Upper Arm P", 0.00025));//0.00005);
    m_lowerPID.setI(SmartDashboard.getNumber("Upper Arm I", 0.00001));//0.00000025);//0.000000001);
    m_lowerPID.setD(SmartDashboard.getNumber("Upper Arm D", 0.0));//0.005);//0.01);//0);//0.0000005);
    m_lowerPID.setIZone(0.001);
    m_lowerPID.setFF(0.000156);
    m_lowerPID.setOutputRange(-1.0, 1.0);
    // TODO: configure max velocity and accel (and move to constants?)
    m_lowerPID.setSmartMotionMaxVelocity(2500, 0);
    m_lowerPID.setSmartMotionMaxAccel(15000, 0);

    m_upperPID.setP(SmartDashboard.getNumber("Lower Arm P", 0.00025));//0.00005);
    m_upperPID.setI(SmartDashboard.getNumber("Lower Arm I", 0.00001));//0.00000025);//0.000000001);
    m_upperPID.setD(SmartDashboard.getNumber("Lower Arm D", 0.0));//0.005);//0.01);//0);//0.0000005);
    m_upperPID.setIZone(0.001);
    m_upperPID.setFF(0.000156);
    m_upperPID.setOutputRange(-1.0, 1.0);
    m_upperPID.setSmartMotionMaxVelocity(2500, 0);
    m_upperPID.setSmartMotionMaxAccel(15000, 0);

    m_extensionPID.setP(SmartDashboard.getNumber("Extension P", 0.00005));
    m_extensionPID.setI(SmartDashboard.getNumber("Extension I", 0.000000001));
    m_extensionPID.setD(SmartDashboard.getNumber("Extension D", 0.0000005));
    m_extensionPID.setIZone(0);
    m_extensionPID.setFF(0.000156);
    m_extensionPID.setOutputRange(-1.0, 1.0);
    m_extensionPID.setSmartMotionMaxVelocity(2500, 0);
    m_extensionPID.setSmartMotionMaxAccel(15000, 0);
  }

  public void setArmOffsets(){
    // TODO: move to encoder config (won't need to call in RobotContainer either)
    m_lowerMotor.getEncoder().setPosition(getLowerAngleAbsolute());
    m_upperMotor.getEncoder().setPosition(getUpperArmAngleAbsolute());
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

  /** Control the arm using the operator controller. */
  public void manualControl() {
    // set speeds of arm motors
    m_lowerMotor.set(MathUtil.applyDeadband(
      -m_operatorController.getLeftY(), Xbox.JOYSTICK_DEADBAND));
    m_upperMotor.set(MathUtil.applyDeadband(
      -m_operatorController.getRightY(), Xbox.JOYSTICK_DEADBAND));
    // set extension motor
    if (m_operatorController.getXButton()) {
      m_extensionMotor.set(Arm.MANUAL_EXTENSION_SPEED);
    } else if (m_operatorController.getYButton()) {
      m_extensionMotor.set(-Arm.MANUAL_EXTENSION_SPEED);
    } else {
      m_extensionMotor.disable();
    }
  }

  public void setGripper(Value value){
    m_gripperSolenoid.set(value);
  }


  // TODO: javadocs and order functions properly
  public void setLowerTargetAngle(double angle) {
    m_lowerPID.setReference(angle, ControlType.kSmartMotion);
  }

  public void setUpperTargetAngle(double angle) {
    m_upperPID.setReference(angle, ControlType.kSmartMotion);
  }

  public void setExtensionTargetLength(double length) {
    m_extensionPID.setReference(length, ControlType.kSmartMotion);
  }

  public void holdPosition() {
    // TODO: need a way to hold the current position
    // TODO: need a function to stop all motors
    // m_lowerPID.setReference(low_angle, ControlType.kSmartMotion);
    // m_upperPID.setReference(high_angle, ControlType.kSmartMotion);
    // m_extensionPID.setReference(length, ControlType.kSmartMotion);
  }
}
