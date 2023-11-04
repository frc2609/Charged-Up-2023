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

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Timer;
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
import frc.robot.utils.ArmFeedForward;
import frc.robot.utils.ArmKinematics;
import frc.robot.utils.BeaverLogger;
import frc.robot.utils.SimPID;
import frc.robot.utils.TunableNumber;

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

  private final SimPID lowerPID = new SimPID(0.15, 0, 0, Tolerances.lowerErrorEpsilon);
  private final SimPID upperPID = new SimPID(0.2, 0.1, 0, Tolerances.upperErrorEpsilon);
  private ArmFeedForward lowerFF = new ArmFeedForward(0.03, 0.0);
  private ArmFeedForward upperFF = new ArmFeedForward(-0.015, 0.0);

  private final TunableNumber lowerPID_P = new TunableNumber("arm/pid/lower_p", lowerPID.getP());
  private final TunableNumber lowerPID_I = new TunableNumber("arm/pid/lower_i", lowerPID.getI());
  private final TunableNumber lowerPID_D = new TunableNumber("arm/pid/lower_d", lowerPID.getD());
  private final TunableNumber lowerF_s = new TunableNumber("arm/pid/lower_ks", lowerFF.ks);
  private final TunableNumber lowerF_v = new TunableNumber("arm/pid/lower_kv", lowerFF.kv);

  private final TunableNumber upperPID_P = new TunableNumber("arm/pid/upper_p", upperPID.getP());
  private final TunableNumber upperPID_I = new TunableNumber("arm/pid/upper_i", upperPID.getI());
  private final TunableNumber upperPID_D = new TunableNumber("arm/pid/upper_d", upperPID.getD());
  private final TunableNumber upperF_s = new TunableNumber("arm/pid/upper_ks", upperFF.ks);
  private final TunableNumber upperF_v = new TunableNumber("arm/pid/upper_kv", upperFF.kv);

  private final SparkMaxPIDController extensionPID = extensionMotor.getPIDController();

  private double lowerSetpoint, upperSetpoint, extensionSetpoint = 0;

  public boolean isEnabled = false;
  public double[][] currentPath = new double[][] {{0.0,0.0,0.0},{0.0,0.0,0.0}};
  public double startTime;
  public boolean isReverse = false;

  public final BeaverLogger logger;

  private final Loop loop = new Loop() {
    int i = 0;
    
    @Override
    public void onStart() {
      DataLogManager.log("Starting Arm Loops");
      logger.addLoggable("Arm Loop Path Length", () -> (double) currentPath.length, false);
      logger.addLoggable("Arm Loop Index", () -> (double) i, false);
      holdPosition();
    }

    @Override
    public void onLoop() {
      synchronized (Arm.this) {
        if (isEnabled) {
          System.out.println(Timer.getFPGATimestamp() - startTime);
          System.out.println("ARM LOOP RUNNING path  len: " + Integer.toString(currentPath.length));
          System.out.println("I = " + Integer.toString(i));
          i = (int) (Math.ceil((Timer.getFPGATimestamp() - startTime) * 50.0)); // 50 loops per second = 0.02 seconds per loop
          if (i <= currentPath.length - 1) {
            // while there are still setpoints left
            if (isReverse) {
              setLowerAngle(Rotation2d.fromDegrees(currentPath[getReverseIndex(i)][0]));
              setUpperAngle(Rotation2d.fromDegrees(currentPath[getReverseIndex(i)][1]));
              setExtensionLength(currentPath[getReverseIndex(i)][2]);
              // log(currentPath[getReverseIndex(i)]);
            } else {
              setLowerAngle(Rotation2d.fromDegrees(currentPath[i][0]));
              setUpperAngle(Rotation2d.fromDegrees(currentPath[i][1]));
              setExtensionLength(currentPath[i][2]);
              // log(currentPath[i]);
            }
          } else {
            // stop the profile when the end is reached
            isEnabled = false;
            i = 0;
          }
        }
        setArmMotors();
        logger.logAll();
      }
    }

    @Override
    public void onStop() {
      DataLogManager.log("Ending Arm Loops");
    }

    public int getReverseIndex(int i) {
      return (currentPath.length - 1) - i;
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

    configureLoggedData();
    configureMotors();
    configurePIDs();

    lowerSetpoint = getLowerAngle().getDegrees();
    upperSetpoint = getUpperAngle().getDegrees();
    extensionSetpoint = getExtensionDistance();
  }

  @Override
  public void periodic() {
    // mechanism2d
    lowerArmLigament.setAngle(getLowerAngle());
    // mechanism2d expects 0 degrees to be in a different place than our encoder
    upperArmLigament.setAngle(Rotation2d.fromDegrees(getUpperAngle().getDegrees() - 90.0));
    upperArmLigament.setLength(getUpperArmLength());

    if (lowerPID_P.hasChanged() || lowerPID_I.hasChanged() || lowerPID_D.hasChanged()) {
      lowerPID.setConstants(lowerPID_P.get(), lowerPID_I.get(), lowerPID_P.get());
    }

    if (lowerF_s.hasChanged() || lowerF_v.hasChanged()) {
      lowerFF = new ArmFeedForward(lowerF_s.get(), lowerF_v.get());
    }

    if (upperPID_P.hasChanged() || upperPID_I.hasChanged() || upperPID_D.hasChanged()) {
      upperPID.setConstants(upperPID_P.get(), upperPID_I.get(), upperPID_D.get());
    }

    if (upperF_s.hasChanged() || upperF_v.hasChanged()) {
      upperFF = new ArmFeedForward(upperF_s.get(), upperF_v.get());
    }
  }

  private void configureLoggedData() {
    // motor applied duty
    logger.addLoggable("arm/lower/applied_output", lowerMotor::getAppliedOutput, true);
    logger.addLoggable("arm/upper/applied_output", upperMotor::getAppliedOutput, true);
    logger.addLoggable("arm/extension/applied_output", extensionMotor::getAppliedOutput, true);
    // motor output
    logger.addLoggable("arm/lower/current", lowerMotor::getOutputCurrent, false);
    logger.addLoggable("arm/upper/current", upperMotor::getOutputCurrent, false);
    logger.addLoggable("arm/extension/current", extensionMotor::getOutputCurrent, false);
    // motor abs position
    logger.addLoggable("arm/lower/position", () -> getLowerAngle().getDegrees(), true);
    logger.addLoggable("arm/upper/position", () -> getUpperAngle().getDegrees(), true);
    logger.addLoggable("arm/extension/position", this::getExtensionDistance, true);
    // motor setpoint
    logger.addLoggable("arm/lower/setpoint", () -> lowerSetpoint, true);
    logger.addLoggable("arm/upper/setpoint", () -> upperSetpoint, true);
    logger.addLoggable("arm/extension/setpoint", () -> extensionSetpoint, true);
    // absolute encoder raw position
    logger.addLoggable("arm/lower/position_raw", lowerEncoder::getRawValue, true);
    logger.addLoggable("arm/upper/position_raw", upperEncoder::getRawValue, true);
    // gravity
    logger.addLoggable("arm/lower/gravity", () -> ArmKinematics.gravitationalTorques(getLowerAngle().getDegrees(), getUpperAngle().getDegrees(), getExtensionDistance())[0], true);
    logger.addLoggable("arm/upper/gravity", () -> ArmKinematics.gravitationalTorques(getLowerAngle().getDegrees(), getUpperAngle().getDegrees(), getExtensionDistance())[1], true);
    logger.addLoggable("arm/extension/gravity", () -> ArmKinematics.gravitationalTorques(getLowerAngle().getDegrees(), getUpperAngle().getDegrees(), getExtensionDistance())[2], true);
  }

  private void configureMotors() {
    lowerMotor.restoreFactoryDefaults();
    upperMotor.restoreFactoryDefaults();
    extensionMotor.restoreFactoryDefaults();

    lowerMotor.enableSoftLimit(SoftLimitDirection.kForward, false);
    lowerMotor.enableSoftLimit(SoftLimitDirection.kReverse, false);
    lowerMotor.setSoftLimit(SoftLimitDirection.kForward, SoftStop.LOWER_FORWARD);
    lowerMotor.setSoftLimit(SoftLimitDirection.kReverse, SoftStop.LOWER_REVERSE);
    lowerMotor.setIdleMode(IdleMode.kBrake);
    lowerMotor.setSmartCurrentLimit(frc.robot.Constants.CurrentLimits.lowerArm);
    lowerMotor.setInverted(IsInverted.LOWER_MOTOR);

    upperMotor.enableSoftLimit(SoftLimitDirection.kForward, false);
    upperMotor.enableSoftLimit(SoftLimitDirection.kReverse, false);
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

    extensionEncoder.setPositionConversionFactor(Encoder.extensionPositionConversion);
    extensionEncoder.setVelocityConversionFactor(Encoder.extensionVelocityConversion);
    extensionEncoder.setPosition(0.0);
  }
  
  private void configurePIDs() {
    lowerPID.setMaxOutput(12); // volts
    lowerPID.setDoneRange(Tolerances.lowerAngle);
    // lowerPID.setMinDoneCycles(0);

    upperPID.setMaxOutput(12); // volts
    upperPID.setDoneRange(Tolerances.upperAngle);
    // upperPID.setMinDoneCycles(0);
    upperPID.setErrorIncrement(0.5);

    extensionPID.setP(0);
    extensionPID.setI(0);
    extensionPID.setD(0);
    extensionPID.setIZone(0.5);
    extensionPID.setFF(0.001);
    extensionPID.setOutputRange(-1.0, 1.0);
    extensionPID.setSmartMotionMaxVelocity(11000, 0); // NEO 550 free rpm
    extensionPID.setSmartMotionMaxAccel(15000, 0);
  }

  private void setArmMotors() {
    // we use feedforward on the current position; feedforward will hold the current position and is never affected by the setpoint
    double[] gravity = ArmKinematics.gravitationalTorques(getLowerAngle().getDegrees(), getUpperAngle().getDegrees(), getExtensionDistance());
    double lowerOutput = lowerPID.calcPID(getLowerAngle().getDegrees()) - lowerFF.calculate(gravity[0]);
    double upperOutput = upperPID.calcPID(getUpperAngle().getDegrees()) + upperFF.calculate(gravity[1]);

    lowerMotor.setVoltage(lowerOutput);
    upperMotor.setVoltage(upperOutput);
  }

  public Loop getLoop() {
    return this.loop;
  }

  /**
   * Returns the amount the arm extension has extended.
   * @return The amount the arm extension has extended in metres.
   */
  public double getExtensionDistance() {
    return extensionEncoder.getPosition();
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

  /**
   * Returns the angle of the upper arm relative to the lower arm.
   * @return The angle of the upper arm relative to the lower arm as a Rotation2d.
   */
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
    return lowerPID.isDone();
  }

  public boolean upperAtSetpoint() {
    return upperPID.isDone();
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
    lowerPID.setDesiredValue(lowerSetpoint);
  }

  public void setUpperAngle(Rotation2d angle) {
    upperSetpoint = angle.getDegrees();
    upperPID.setDesiredValue(upperSetpoint);
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

  // PID & FF overrides this so it won't work until you disable them
  // public void openLoopSetVoltage(double lower, double upper, double extension) {
  //   lowerMotor.setVoltage(lower*6);
  //   upperMotor.setVoltage(upper*6);
  //   extensionMotor.setVoltage(extension*6);
  // }

  public void stopAllMotors() {
    lowerMotor.stopMotor();
    upperMotor.stopMotor();
    extensionMotor.stopMotor();
  }
}