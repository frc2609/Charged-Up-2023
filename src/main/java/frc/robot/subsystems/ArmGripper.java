// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;

import java.util.logging.Logger;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.MP.Loop;
import frc.robot.Constants.Arm;
import frc.robot.Constants.CANID;
import frc.robot.Constants.DIO;
import frc.robot.Constants.Limits;
import frc.robot.Constants.Xbox;
import frc.robot.Constants.Arm.Encoder;
import frc.robot.Constants.Arm.IsInverted;
import frc.robot.Constants.Arm.Pneumatics;
import frc.robot.Constants.Arm.SoftStop;
import frc.robot.utils.BeaverLogger;

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
  private final DutyCycleEncoder m_lowerEncoderAbsoluteBak = new DutyCycleEncoder(DIO.ARM_LOWER_ENCODER_BAK);
  private final DutyCycleEncoder m_upperEncoderAbsolute = new DutyCycleEncoder(DIO.ARM_UPPER_ENCODER);
  private final DutyCycleEncoder m_upperEncoderAbsoluteBak = new DutyCycleEncoder(DIO.ARM_UPPER_ENCODER_BAK);

  private final RelativeEncoder m_lowerEncoderRelative = m_lowerMotor.getEncoder();
  private final RelativeEncoder m_upperEncoderRelative = m_upperMotor.getEncoder();
  private final RelativeEncoder m_extensionEncoderRelative = m_extensionMotor.getEncoder();

  private final SparkMaxPIDController m_lowerPID = m_lowerMotor.getPIDController();
  private final SparkMaxPIDController m_upperPID = m_upperMotor.getPIDController();
  private final SparkMaxPIDController m_extensionPID = m_extensionMotor.getPIDController();

  public boolean isMP = false;
  public double[][] currentPath = new double[][]{{0.0,0.0,0.0},{0.0,0.0,0.0}};
  public double startTime;
  public boolean isReverse = false;

  public int getReverseIndex(int i){
    return (currentPath.length-1)-i;
  }
  private final Loop m_loop = new Loop(){
    int i = 0;
    ArmGripper _arm;
    @Override
    public void onStart() {
      System.out.println("Starting ArmGripper Loops");
    }

    @Override
    public void onLoop() {
      synchronized (ArmGripper.this){
        if(isMP){
          i=(int) (Math.ceil((Timer.getFPGATimestamp()-startTime)*50)); // 50 loops per second = 0.02 seconds per loop
          System.out.println("ARM LOOP RUNNING path  len: " + Integer.toString(currentPath.length));
          System.out.println("I = " + Integer.toString(i));
          if(i <= currentPath.length-1){
            if(isReverse){
              setLowerTargetAngle(currentPath[getReverseIndex(i)][0]);
              setUpperTargetAngle(currentPath[getReverseIndex(i)][1]);
              setExtensionTargetLength(currentPath[getReverseIndex(i)][2]);
              System.out.println(currentPath[i]);
              log(currentPath[getReverseIndex(i)]);
            }else{
              setLowerTargetAngle(currentPath[i][0]);
              setUpperTargetAngle(currentPath[i][1]);
              setExtensionTargetLength(currentPath[i][2]);
              System.out.println(currentPath[i]);
              log(currentPath[i]);
            }
          }else{
            isMP = false;
            i = 0;
          }

        }else{
          i = 0;
        }
      }
      
    }

    @Override
    public void onStop() {
      System.out.println("Ending ArmGripper Loops");
    }
    
  };

  private DigitalInput intakeSensor = new DigitalInput(DIO.INTAKE_SENSOR); 
  private boolean m_isCubeRequested;

  /** @deprecated Use commands to control this subsystem instead. */
  private final XboxController m_operatorController;

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
  public Loop getLoop(){
		return m_loop;
	}
  public void log(double[] joint_targets){
    BeaverLogger.getInstance().logArm(joint_targets, this);
  }

  @Override
  public void periodic() {
    // intake sensor
    SmartDashboard.putBoolean("intakeSensor", intakeSensor.get());
    // extension
    SmartDashboard.putNumber("Extension Arm RPM", m_extensionEncoderRelative.getVelocity());
    // absolute encoder value
    SmartDashboard.putNumber("Lower Arm Position Bak (0-1)", m_lowerEncoderAbsoluteBak.getAbsolutePosition());
    SmartDashboard.putNumber("Lower Arm Position (0-1)", m_lowerEncoderAbsolute.getAbsolutePosition());
    SmartDashboard.putNumber("Upper Arm Position (0-1)", m_upperEncoderAbsolute.getAbsolutePosition());
    // absolute angle
    SmartDashboard.putNumber("Lower Arm Angle Bak (Deg)", getLowerAngleAbsoluteBak()); // positive away from robot
    SmartDashboard.putNumber("Lower Arm Angle (Deg)", getLowerAngleAbsolute()); // positive away from robot
    SmartDashboard.putNumber("Upper Arm Angle (Deg)", getUpperAngleAbsolute()); // positive away from robot
    SmartDashboard.putNumber("Upper Arm Angle Bak (Deg)", getUpperAngleAbsoluteBak()); // positive away from robot
    
    // relative angle
    SmartDashboard.putNumber("Lower Arm NEO Encoder Position", getLowerAngleRelative());
    SmartDashboard.putNumber("Upper Arm NEO Encoder Position", getUpperAngleRelative());
    // lengths
    SmartDashboard.putNumber("Lower Arm Length (m)", getLowerArmLength());
    SmartDashboard.putNumber("Upper Arm Base Length (m)", getUpperArmBaseLength());
    SmartDashboard.putNumber("Upper Arm Total Length (m)", getUpperArmTotalLength());
    SmartDashboard.putNumber("Arm Extension Distance (m)", getExtensionDistance());
    // temperatures
    SmartDashboard.putNumber("Lower Arm Motor Temp (C)", m_lowerMotor.getMotorTemperature());
    SmartDashboard.putNumber("Upper Arm Motor Temp (C)", m_upperMotor.getMotorTemperature());
    SmartDashboard.putNumber("Extension Arm Motor Temp (C)", m_extensionMotor.getMotorTemperature());
    // check solenoid status
    if (m_gripperSolenoid.isFwdSolenoidDisabled()) {
      System.out.print("OPEN SOLENOID DISABLED: CHECK FOR SHORTED/DISCONNECTED WIRES");
    }
    if (m_gripperSolenoid.isRevSolenoidDisabled()) {
      System.out.print("CLOSE SOLENOID DISABLED: CHECK FOR SHORTED/DISCONNECTED WIRES");
    }
  }

  private void configureEncoders() {
    // position
    m_lowerEncoderRelative.setPositionConversionFactor(Encoder.LOWER_POSITION_CONVERSION);
    m_upperEncoderRelative.setPositionConversionFactor(Encoder.UPPER_POSITION_CONVERSION);
    m_extensionEncoderRelative.setPositionConversionFactor(Encoder.EXTENSION_POSITION_CONVERSION);
    // velocity
    m_lowerEncoderRelative.setVelocityConversionFactor(Encoder.LOWER_VELOCITY_CONVERSION);
    m_upperEncoderRelative.setVelocityConversionFactor(Encoder.UPPER_VELOCITY_CONVERSION);
    m_extensionEncoderRelative.setVelocityConversionFactor(Encoder.EXTENSION_VELOCITY_CONVERSION);
    // Copy absolute position to NEO encoders
    m_lowerEncoderRelative.setPosition(getLowerAngleAbsolute());
    m_upperEncoderRelative.setPosition(getUpperAngleAbsolute());
    // Reset extension encoder
    m_extensionEncoderRelative.setPosition(0.0);
  }

  private void configureMotors() {
    m_lowerMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
    m_lowerMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
    m_lowerMotor.setSoftLimit(SoftLimitDirection.kForward, SoftStop.LOWER_FORWARD); // degrees
    m_lowerMotor.setSoftLimit(SoftLimitDirection.kReverse, SoftStop.LOWER_REVERSE); // degrees
    m_lowerMotor.setIdleMode(IdleMode.kBrake);
    m_lowerMotor.setSmartCurrentLimit(frc.robot.Constants.Limits.LOWER_ARM_CURRENT);
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
    m_lowerPID.setP(0.0);
    m_lowerPID.setI(0.00015);//0.00015
    m_lowerPID.setD(0.0);
    m_lowerPID.setIZone(2);
    m_lowerPID.setFF(0.0002);
    m_lowerPID.setOutputRange(-1,1);
    m_lowerPID.setSmartMotionMaxVelocity(3500, 0);
    m_lowerPID.setSmartMotionMaxAccel(4000, 0);

    m_upperPID.setP(0.0);
    m_upperPID.setI(0.0);
    m_upperPID.setD(0.0);
    m_upperPID.setIZone(0.5);
    m_upperPID.setFF(0.0001);
    m_upperPID.setOutputRange(-0.75, 0.75);
    m_upperPID.setSmartMotionMaxVelocity(3500, 0);
    m_upperPID.setSmartMotionMaxAccel(6000, 0);

    m_extensionPID.setP(0);
    m_extensionPID.setI(0);
    m_extensionPID.setD(0);
    m_extensionPID.setIZone(0.5);
    m_extensionPID.setFF(0.001);
    m_extensionPID.setOutputRange(-1.0, 1.0);
    m_extensionPID.setSmartMotionMaxVelocity(11000, 0); // NEO 550 free rpm
    m_extensionPID.setSmartMotionMaxAccel(15000, 0);
  }

  /**
   * Returns the amount the arm extension has extended.
   * @return The amount the arm extension has extended in metres.
   */
  public double getExtensionDistance() {
    return m_extensionEncoderRelative.getPosition();
  }

  /**
   * Returns the velocity of the upper arm extension.
   * @return The velocity of the upper arm extension in m/s.
   */
  public double getExtensionVelocity() {
    return m_extensionEncoderRelative.getVelocity();
  }

  /**
   * Returns the output of the intake sensor.
   * @return Boolean true if there is a game piece in the gripper false otherwise.
   */
  public boolean getIntakeSensor() {
    return intakeSensor.get();
  }

  /**
   * Returns the angle of the lower arm relative to the front of the robot
   * using the absolute encoder.
   * <p>WARNING: This value is only accurate for certain arm positions (mostly
   * when the arm is facing upwards). Only use this reading to offset the NEO
   * encoders at the start of the match!
   * @return The robot-relative angle of the lower arm in degrees.
   */
  private double getLowerAngleAbsolute() {
    // Adds 90 degrees because the offset was measured at a 90 degree angle. <- this is incorrect, why do we add 90?
    return ((m_lowerEncoderAbsolute.getAbsolutePosition() - Encoder.LOWER_POSITION_OFFSET) * 360) + 90.0;
  }

  /**
   * Returns the angle of the lower arm relative to the front of the robot
   * using the absolute encoder.
   * <p>WARNING: This value is only accurate for certain arm positions (mostly
   * when the arm is facing upwards). Only use this reading to offset the NEO
   * encoders at the start of the match!
   * @return The robot-relative angle of the lower arm in degrees.
   */
  private double getLowerAngleAbsoluteBak() {
    // Adds 90 degrees because the offset was measured at a 90 degree angle. <- this is incorrect, why do we add 90?
    return ((m_lowerEncoderAbsoluteBak.getAbsolutePosition() - Encoder.LOWER_POSITION_OFFSET_BAK) * Encoder.LOWER_BAK_ABSOLUTE_POSITION_CONVERSION) + 90.0;
  }

  /**
   * Returns the angle of the lower arm relative to the front of the robot
   * using the absolute encoder.
   * @return The robot-relative angle of the lower arm in degrees.
   */
  public double getLowerAngleRelative() {
    return m_lowerEncoderRelative.getPosition();
  }

  /**
   * Returns the angular velocity of the lower arm joint.
   * @return The angular velocity of the lower arm joint in degrees per second.
   */
  public double getLowerJointAngularVelocity() {
    return m_lowerEncoderRelative.getVelocity();
  }

  /**
   * Returns the length of the lower arm.
   * @return The length of the lower arm in metres.
   */
  public double getLowerArmLength() {
    return Arm.LOWER_ARM_LENGTH;
  }

  /**
   * Returns the angle of the upper arm relative to the lower arm in degrees.
   * <p>WARNING: This value is only accurate for certain arm positions (mostly
   * when the arm is facing upwards). Only use this reading to offset the NEO
   * encoders at the start of the match!
   * @return The robot-relative angle of the upper arm in degrees.
   */
  private double getUpperAngleAbsoluteBak() {
    // Adds 90 degrees because the offset was measured at a 90 degree angle. <- this is incorrect, why do we add 90?
    return ((m_upperEncoderAbsoluteBak.getAbsolutePosition() - Encoder.UPPER_POSITION_OFFSET_BAK)  * Encoder.LOWER_BAK_ABSOLUTE_POSITION_CONVERSION) + 90;
    // return 0;
  }

    /**
   * Returns the angle of the upper arm relative to the lower arm in degrees.
   * <p>WARNING: This value is only accurate for certain arm positions (mostly
   * when the arm is facing upwards). Only use this reading to offset the NEO
   * encoders at the start of the match!
   * @return The robot-relative angle of the upper arm in degrees.
   */
  private double getUpperAngleAbsolute() {
    // Adds 90 degrees because the offset was measured at a 90 degree angle. <- this is incorrect, why do we add 90?
    return ((m_upperEncoderAbsolute.getAbsolutePosition() - Encoder.UPPER_POSITION_OFFSET) * 360) + 90.0;
    // return 0;
  }

  /**
   * Returns the angle of the upper arm relative to the front of the robot using the absolute encoder.
   * @return The robot-relative angle of the upper arm in degrees.
   */
  public double getUpperAngleRelative() {
    return m_upperEncoderRelative.getPosition();
  }
  
  /**
   * Returns the angular velocity of the upper arm joint.
   * @return The angular velocity of the upper arm joint in degrees per second.
   */
  public double getUpperJointAngularVelocity() {
    return m_upperEncoderRelative.getVelocity();
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
    m_lowerPID.setReference(getLowerAngleRelative(), ControlType.kSmartMotion);
    m_upperPID.setReference(getUpperAngleRelative(), ControlType.kSmartMotion);
    m_extensionPID.setReference(getExtensionDistance(), ControlType.kSmartMotion);
  }

  /** 
   * Control the arm position using the operator controller.
   * Does not control the gripper solenoids.
   * @deprecated Use either {@link ManualArmControl} command instead.
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
    System.out.println(("gripper oppened at Lower: "+Double.toString(getLowerAngleRelative())+" upper: "+ Double.toString(getUpperAngleRelative())+" extension:"+Double.toString(getExtensionDistance())));
    m_gripperSolenoid.set(kForward);
  }

  public void closeGripper() {
    System.out.println(("gripper close at Lower: "+Double.toString(getLowerAngleRelative())+" upper: "+ Double.toString(getUpperAngleRelative())+" extension:"+Double.toString(getExtensionDistance())));
    m_gripperSolenoid.set(kReverse);
  }

  public void requestCube(){
    m_isCubeRequested = true;
    // LED.setCube();
  }
  
  public void requestCone(){
    m_isCubeRequested = false;
    // LED.setCone();
  }

  public void setBrake(boolean isBrake) {
    if (isBrake) {
      m_lowerMotor.setIdleMode(IdleMode.kBrake);
      m_upperMotor.setIdleMode(IdleMode.kBrake);
      m_extensionMotor.setIdleMode(IdleMode.kBrake);
    } else {
      m_lowerMotor.setIdleMode(IdleMode.kCoast);
      m_upperMotor.setIdleMode(IdleMode.kCoast);
      m_extensionMotor.setIdleMode(IdleMode.kCoast);
    }
  }

  /** Set the lower arm angle relative to the front of the robot in degrees. */
  public void setLowerTargetAngle(double angle) {
    m_lowerPID.setFF(0.0001+Math.abs(Math.cos(Math.toRadians(getLowerAngleRelative()))*0.0001));
    m_lowerPID.setReference(angle, ControlType.kSmartMotion);
  }

  /** Set the upper arm angle relative to the front of the robot in degrees. */
  public void setUpperTargetAngle(double angle) {
    m_upperPID.setFF(0.0001+Math.abs(Math.cos(Math.toRadians(getUpperAngleRelative()))*0.0002));
    m_upperPID.setReference(angle, ControlType.kSmartMotion);
  }

  /** Set the amount to extend the upper arm in metres. */
  public void setExtensionTargetLength(double length) {
    m_extensionPID.setReference(length, ControlType.kSmartMotion);
  }

  /**
   * Set the lower and upper arm relative encoders to the position reported by
   * the absolute encoders.
   */
  public void setEncoderOffsets() {
    System.out.println("Old Lower Value: " + getLowerAngleAbsolute());
    m_lowerEncoderRelative.setPosition(getLowerAngleAbsolute());
    System.out.println("Old Upper Value: " + getUpperAngleAbsolute());
    m_upperEncoderRelative.setPosition(getUpperAngleAbsolute());
    // TODO: should this reset the extension encoder (probably)
  }

  public void stopAllMotors() {
    m_lowerMotor.stopMotor();
    m_upperMotor.stopMotor();
    m_extensionMotor.stopMotor();
  }
}
