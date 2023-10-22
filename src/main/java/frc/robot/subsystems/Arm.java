// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.MP.Loop;
import frc.robot.Constants.CANID;
import frc.robot.Constants.DIO;
import frc.robot.Constants.CurrentLimits;
import frc.robot.Constants.Arm.Encoder;
import frc.robot.Constants.Arm.IsInverted;
import frc.robot.Constants.Arm.Measurements;
import frc.robot.Constants.Arm.SoftStop;
import frc.robot.Constants.Arm.Tolerances;
import frc.robot.utils.AbsoluteEncoderHandler;
import frc.robot.utils.ArmKinematics;
import frc.robot.utils.BeaverLogger;

public class Arm extends SubsystemBase {
  private final CANSparkMax lowerMotor = new CANSparkMax(CANID.lowerArmMotor, MotorType.kBrushless);
  private final CANSparkMax upperMotor = new CANSparkMax(CANID.upperArmMotor, MotorType.kBrushless);
  private final CANSparkMax extensionMotor = new CANSparkMax(CANID.extensionMotor, MotorType.kBrushless);

  private final AbsoluteEncoderHandler lowerEncoder, upperEncoder, lowerBackupEncoder, upperBackupEncoder;
  private final RelativeEncoder extensionEncoder = extensionMotor.getEncoder();

  private final Mechanism2d armMechanism = new Mechanism2d(4, 4);
  private final MechanismRoot2d mechanismRoot = armMechanism.getRoot("Arm", 1, 1);
  private final MechanismLigament2d lowerArmLigament = mechanismRoot.append(new MechanismLigament2d("Lower Arm", getLowerArmLength(), 0));
  private final MechanismLigament2d upperArmLigament = lowerArmLigament.append(new MechanismLigament2d("Upper Arm + Extension", getUpperArmLength(), 0));

  private final PIDController lowerPID = new PIDController(0, 0, 0);
  private final PIDController upperPID = new PIDController(0, 0, 0);
  private final SimpleMotorFeedforward lowerFF = new SimpleMotorFeedforward(0, 0.01);
  private final SimpleMotorFeedforward upperFF = new SimpleMotorFeedforward(0, 0.01);
  private final SparkMaxPIDController extensionPID = extensionMotor.getPIDController();

  private double lowerSetpoint, upperSetpoint, extensionSetpoint = 0;

  public final BeaverLogger logger;

  // TODO: add this to BeaverLogger
  private final DoubleLogEntry log_tau1 = new DoubleLogEntry(DataLogManager.getLog(), "/arm/gravity/tau1");
  private final DoubleLogEntry log_tau2 = new DoubleLogEntry(DataLogManager.getLog(), "/arm/gravity/tau2");
  private final DoubleLogEntry log_tau3 = new DoubleLogEntry(DataLogManager.getLog(), "/arm/gravity/tau3");

  private final Loop loop = new Loop(){
    int i = 0;
    Arm _arm;
    @Override
    public void onStart() {
      DataLogManager.log("Starting Arm Loops");
    }

    @Override
    public void onLoop() {
      synchronized (Arm.this){
        // arm logic here
      }
    }

    @Override
    public void onStop() {
      DataLogManager.log("Ending Arm Loops");
    }
  };
  
  /** Creates a new Arm. */
  public Arm() {
    SmartDashboard.putData("Arm Mechanism", armMechanism);
    logger = new BeaverLogger();
    
    lowerEncoder = new AbsoluteEncoderHandler(DIO.armLowerEncoder, Encoder.lowerPositionOffset, Encoder.lowerPositionConversion);
    lowerBackupEncoder = new AbsoluteEncoderHandler(DIO.armLowerBackupEncoder, Encoder.lowerBackupPositionOffset, Encoder.lowerBackupPositionConversion);
    upperEncoder = new AbsoluteEncoderHandler(DIO.armUpperEncoder, Encoder.upperPositionOffset, Encoder.upperPositionConversion);
    upperBackupEncoder = new AbsoluteEncoderHandler(DIO.armUpperBackupEncoder, Encoder.upperBackupPositionOffset, Encoder.upperBackupPositionConversion);
    
    extensionEncoder.setPositionConversionFactor(1.0);
    extensionEncoder.setVelocityConversionFactor(Encoder.extensionVelocityConversion);
    extensionEncoder.setPosition(0.0);

    configureMotors();
    configurePIDs();

    // lowerSetpoint = new DoubleLogEntry(log, "/arm/setpoints/lower");
    // upperSetpoint = new DoubleLogEntry(log, "/arm/setpoints/upper");
    // extensionSetpoint = new DoubleLogEntry(log, "/arm/setpoints/extension");

    // lowerAngle = new DoubleLogEntry(log, "/arm/angles/lower");
    // upperAngle = new DoubleLogEntry(log, "/arm/angles/upper");
    // extensionDistance = new DoubleLogEntry(log, "/arm/angles/extension");

    // lowerBackupAngle = new DoubleLogEntry(log, "arm/angles/lower_backup");
    // upperBackupAngle = new DoubleLogEntry(log, "arm/angles/upper_backup");

    // lowerOutput = new DoubleLogEntry(log, "/arm/output/lower");
    // upperOutput = new DoubleLogEntry(log, "/arm/output/upper");
    // extensionOutput = new DoubleLogEntry(log, "/arm/output/extension");

    // lowerCurrent = new DoubleLogEntry(log, "/arm/current/lower");
    // upperCurrent = new DoubleLogEntry(log, "/arm/current/upper");
    // extensionCurrent = new DoubleLogEntry(log, "/arm/current/extension");
    logger.addLoggable("/arm/angles/lower", getLowerAngle()::getDegrees, true);
    logger.addLoggable("/arm/angles/upper", getUpperAngle()::getDegrees, true);
    logger.addLoggable("/arm/angles/extension", this::getExtensionDistance, true);
  }

  public Loop getLoop(){
    return this.loop;
  }

  @Override
  public void periodic() {
    // mechanism2d
    lowerArmLigament.setAngle(getLowerAngle());
    upperArmLigament.setAngle(Rotation2d.fromDegrees(getUpperAngle().getDegrees() + 180.0));
    upperArmLigament.setLength(getUpperArmLength());
    
    double[] gravity = ArmKinematics.gravitationalTorques(getLowerAngle().getDegrees(), getUpperAngle().getDegrees(), getExtensionDistance());
    double lowerOutput = lowerPID.calculate(getLowerAngle().getDegrees()) + lowerFF.calculate(gravity[0]);
    double upperOutput = upperPID.calculate(getUpperAngle().getDegrees()) + upperFF.calculate(gravity[1]);
    this.log_tau1.append(gravity[0]);
    this.log_tau2.append(gravity[1]);
    this.log_tau3.append(gravity[2]);
    lowerMotor.setVoltage(0); // lowerOutput
    upperMotor.setVoltage(0); // upperOutput

    this.logger.logAll();
  }

  private void configureMotors() {
    lowerMotor.restoreFactoryDefaults();
    upperMotor.restoreFactoryDefaults();
    extensionMotor.restoreFactoryDefaults();

    lowerMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
    lowerMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
    lowerMotor.setSoftLimit(SoftLimitDirection.kForward, SoftStop.LOWER_FORWARD);
    lowerMotor.setSoftLimit(SoftLimitDirection.kReverse, SoftStop.LOWER_REVERSE);
    lowerMotor.setIdleMode(IdleMode.kBrake);
    lowerMotor.setSmartCurrentLimit(frc.robot.Constants.CurrentLimits.lowerArm);
    lowerMotor.setInverted(IsInverted.LOWER_MOTOR);

    upperMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
    upperMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
    upperMotor.setSoftLimit(SoftLimitDirection.kForward, SoftStop.UPPER_FORWARD);
    upperMotor.setSoftLimit(SoftLimitDirection.kReverse, SoftStop.UPPER_REVERSE);
    upperMotor.setIdleMode(IdleMode.kBrake);
    upperMotor.setSmartCurrentLimit(CurrentLimits.upperArm);
    upperMotor.setInverted(IsInverted.UPPER_MOTOR);

    extensionMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
    extensionMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
    extensionMotor.setSoftLimit(SoftLimitDirection.kForward, SoftStop.EXTENSION_FORWARD);
    extensionMotor.setSoftLimit(SoftLimitDirection.kReverse, SoftStop.EXTENSION_REVERSE);
    extensionMotor.setIdleMode(IdleMode.kBrake);
    extensionMotor.setSmartCurrentLimit(CurrentLimits.extension);
    extensionMotor.setInverted(IsInverted.EXTENSION_MOTOR);
  }
  
  private void configurePIDs() {
    lowerPID.setTolerance(Tolerances.lowerAngle);
    upperPID.setTolerance(Tolerances.upperAngle);

    extensionPID.setP(0);
    extensionPID.setI(0);
    extensionPID.setD(0);
    extensionPID.setIZone(0.5);
    extensionPID.setFF(0.001);
    extensionPID.setOutputRange(-1.0, 1.0);
    extensionPID.setSmartMotionMaxVelocity(11000, 0); // NEO 550 free rpm
    extensionPID.setSmartMotionMaxAccel(15000, 0);
  }

  /**
   * Returns the amount the arm extension has extended.
   * @return The amount the arm extension has extended in metres.
   */
  public double getExtensionDistance() {
    return extensionEncoder.getPosition() * Encoder.extensionPositionConversion;
  }

  /**
   * Returns the velocity of the upper arm extension.
   * @return The velocity of the upper arm extension in m/s.
   */
  public double getExtensionVelocity() {
    return extensionEncoder.getVelocity();
  }

  public Rotation2d getLowerAngle() {
    return Rotation2d.fromDegrees(lowerEncoder.getPosition());
  }

  public Rotation2d getUpperAngle() {
    return Rotation2d.fromDegrees(upperEncoder.getPosition());
  }

  public Rotation2d getLowerBackupAngle() {
    return Rotation2d.fromDegrees(lowerBackupEncoder.getPosition());
  }

  public Rotation2d getUpperBackupAngle() {
    return Rotation2d.fromDegrees(upperBackupEncoder.getPosition());
  }

  /**
   * Returns the length of the lower arm.
   * @return The length of the lower arm in metres.
   */
  public double getLowerArmLength() {
    return Measurements.lowerArmLength;
  }

  /**
   * Returns the length of the upper arm without the extension.
   * @return The length of the upper arm excluding the extension in metres.
   */
  public double getUpperArmBaseLength() {
    return Measurements.upperArmBaseLength;
  }

  /**
   * Returns the length of the upper arm + the current extension length.
   * @return The length of the upper arm including the extension in metres.
   */
  public double getUpperArmLength() {
    return getExtensionDistance() + getUpperArmBaseLength();
  }

  /**
   * Instruct all motor PID controllers to hold their current position.
   */
  public void holdPosition() {
    setTargets(getLowerAngle(), getUpperAngle(), getExtensionDistance());
  }

  public boolean lowerAtSetpoint() {
    return lowerPID.atSetpoint();
  }

  public boolean upperAtSetpoint() {
    return upperPID.atSetpoint();
  }

  public boolean extensionAtSetpoint() {
    final double extensionError = extensionSetpoint - getExtensionDistance();
    return Math.abs(extensionError) < Tolerances.extensionLength;
  }

  public boolean allAtSetpoint() {
    return lowerAtSetpoint() && upperAtSetpoint() && extensionAtSetpoint();
  }

  public void setBrake(boolean isBrake) {
    if (isBrake) {
      lowerMotor.setIdleMode(IdleMode.kBrake);
      upperMotor.setIdleMode(IdleMode.kBrake);
      extensionMotor.setIdleMode(IdleMode.kBrake);
    } else {
      lowerMotor.setIdleMode(IdleMode.kCoast);
      upperMotor.setIdleMode(IdleMode.kCoast);
      extensionMotor.setIdleMode(IdleMode.kCoast);
    }
  }

  public void setLowerAngle(Rotation2d angle) {
    lowerSetpoint = angle.getDegrees();
    lowerPID.setSetpoint(lowerSetpoint);
  }

  public void setUpperAngle(Rotation2d angle) {
    upperSetpoint = angle.getDegrees();
    upperPID.setSetpoint(upperSetpoint);
  }

  /** Set the amount to extend the upper arm in metres. */
  public void setExtensionLength(double length) {
    extensionSetpoint = length;
    extensionPID.setReference(extensionSetpoint, ControlType.kSmartMotion);
  }

  /**
   * Set the lower and upper angles and extension length.
   * @param lowerAngle Lower target angle.
   * @param upperAngle Upper target angle.
   * @param length Extension length in metres.
   */
  public void setTargets(Rotation2d lowerAngle, Rotation2d upperAngle, double length) {
    setLowerAngle(lowerAngle);
    setUpperAngle(upperAngle);
    setExtensionLength(length);
  }

  public void stopAllMotors() {
    lowerMotor.stopMotor();
    upperMotor.stopMotor();
    extensionMotor.stopMotor();
  }
}