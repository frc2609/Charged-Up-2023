// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// Move ECVT into SwerveModule???
// TODO: swap motors properly

package frc.robot.subsystems;

import static frc.robot.Constants.Swerve.Gains.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.Limits;

// velocity conversion factors with/without motors
// Should this be in its own class?

/** Add your docs here. */
public class SwerveMotorGroup {
  private double prevVel = 0;
  public class ECVT{
    private RelativeEncoder ringEncoder, sunEncoder;
    private final double sunTeeth = 36.0;
    private final double ringInner = 72.0;
    private final double bevelToWheel = 1.0/3.0;
    private final double carrierSpur = 24.0;
    private final double driveSpur = 20.0;
    // motor B ratios
    private final double spurTeeth = 33;
    private final double ringOuter = 66;
    public ECVT(RelativeEncoder ringEncoder, RelativeEncoder sunEncoder){
      this.ringEncoder = ringEncoder;
      this.sunEncoder = sunEncoder;
    }
    public double getOutputSpeed(){
      return ((2.0/15.0)*(ringEncoder.getVelocity()+sunEncoder.getVelocity())*Constants.Swerve.WHEEL_CIRCUMFERENCE)/60.0;
    }
    public double getSunSetpoint(double targetVelocity){
      return (((targetVelocity-((ringEncoder.getVelocity()*(2.0/15.0)*Constants.Swerve.WHEEL_CIRCUMFERENCE)/60.0))*60.0)/Constants.Swerve.WHEEL_CIRCUMFERENCE)*(7.5);
    }
    public double getRingSetpoint(double targetVelocity){
      return (((targetVelocity-((sunEncoder.getVelocity()*(2.0/15.0)*Constants.Swerve.WHEEL_CIRCUMFERENCE)/60.0))*60.0)/Constants.Swerve.WHEEL_CIRCUMFERENCE)*(7.5);
    }
    
    public double SIM_getSunSetpoint(double targetVelocity, double ringVel){
      return (((targetVelocity-((ringVel*(2.0/15.0)*Constants.Swerve.WHEEL_CIRCUMFERENCE)/60.0))*60.0)/Constants.Swerve.WHEEL_CIRCUMFERENCE)*(7.5);
    }
    public double SIM_getOutputSpeed(double sunVel, double ringVel){
      return ((2.0/15.0)*(sunVel+ringVel)*Constants.Swerve.WHEEL_CIRCUMFERENCE)/60.0;
    }
    public double getPositionMeters(){
      return ((2.0/15.0)*(sunEncoder.getPosition()+ringEncoder.getPosition()))*Constants.Swerve.WHEEL_CIRCUMFERENCE;
    }
  }

  public final CANSparkMax m_primaryMotor;
  public final CANSparkMax m_secondaryMotor;
  private boolean isPrimaryOverThreshold;
  private int primarySpeedCounter = 0;
  public final RelativeEncoder m_primaryEncoder;
  public final RelativeEncoder m_secondaryEncoder;
  private final ECVT m_ecvt;
  private double prevTime = -1;
  private final SlewRateLimiter m_torqueRateLimiter = new SlewRateLimiter(4, -2, 0);

  private final PIDController m_primaryPID =
      new PIDController(drivePID_kP, drivePID_kI, drivePID_kD);

  private final SimpleMotorFeedforward m_primaryFF =
      new SimpleMotorFeedforward(driveFF_kS, driveFF_kV, driveFF_kA);

  private final String m_name;

  public SwerveMotorGroup(
    int primaryDriveMotorID,
    int secondaryDriveMotorID,
    boolean invertDriveMotors,
    String name
  ) {
    m_primaryMotor = new CANSparkMax(primaryDriveMotorID, MotorType.kBrushless);
    m_secondaryMotor = new CANSparkMax(secondaryDriveMotorID, MotorType.kBrushless);
    m_primaryEncoder = m_primaryMotor.getEncoder();
    m_secondaryEncoder = m_secondaryMotor.getEncoder();
    m_primaryEncoder.setVelocityConversionFactor(1);
    m_secondaryEncoder.setVelocityConversionFactor(1);
    m_primaryEncoder.setPositionConversionFactor(1);
    m_secondaryEncoder.setPositionConversionFactor(1);
    m_primaryMotor.setInverted(invertDriveMotors);
    m_secondaryMotor.setInverted(invertDriveMotors);
    m_primaryMotor.setIdleMode(IdleMode.kBrake);
    m_secondaryMotor.setIdleMode(IdleMode.kBrake);
    m_primaryMotor.setSmartCurrentLimit(Limits.DRIVE_PRIMARY_CURRENT);
    m_secondaryMotor.setSmartCurrentLimit(Limits.DRIVE_SECONDARY_CURRENT);
    m_ecvt = new ECVT(m_secondaryEncoder, m_primaryEncoder);
    m_name = name;
    
    SmartDashboard.putNumber("Test Secondary RPM", 0);
    SmartDashboard.putNumber("Test Primary RPM", 0);
    SmartDashboard.putNumber("Test Target Vel", 1);
  }

  /** 
   * Returns the distance driven by the drive motors.
   * 
   * @return How far the drive motors have travelled in metres.
   */
  public double getPosition() {
    return m_ecvt.getPositionMeters();
  }

  /** 
   * Returns the current wheel velocity.
   * 
   * @return How fast the wheel is moving in metres per second.
   */
  public double getVelocity() {
    return m_ecvt.getOutputSpeed();
  }

  /**
   * Reset the drive encoder positions.
   */
  public void resetEncoders() {
    m_primaryEncoder.setPosition(0);
    m_secondaryEncoder.setPosition(0);
  }

  /**
   * TODO: javadoc
   */
  public void simulateECVT() {
    SmartDashboard.putNumber("Output SIM", m_ecvt.SIM_getOutputSpeed(SmartDashboard.getNumber("Test Primary RPM", 0), SmartDashboard.getNumber("Test Secondary RPM", 0)));
    SmartDashboard.putNumber("SunSetp SIM", m_ecvt.SIM_getSunSetpoint(SmartDashboard.getNumber("Test Target Vel", 0), SmartDashboard.getNumber("Test Secondary RPM", 0)));
    SmartDashboard.putNumber("position", m_ecvt.getPositionMeters());
  }

  /**
   * TODO: describe
   * 
   * @param speedMetersPerSecond The target speed in metres per second.
   * @param boostThrottle Boost throttle (0 to 1).
   * @param maxSpeedEnabled Whether or not to use boost.
   */
  public void set(double speedMetersPerSecond, double boostThrottle, double torqueThrottle, boolean maxSpeedEnabled) {
    // TODO: rewrite this function
    // Calculate the drive output from the drive PID controller.
    
    // m_primaryPID.setP(Constants.Swerve.Gains.drivePID_kP_auto);
    // m_primaryPID.setI(Constants.Swerve.Gains.drivePID_kI_auto);
    // m_primaryPID.setD(Constants.Swerve.Gains.drivePID_kD_auto);
    // final double driveOutput =
    // m_primaryPID.calculate(m_secondaryEncoder.getVelocity(), m_ecvt.getRingSetpoint(speedMetersPerSecond));;//m_primaryEncoder.getVelocity(), speedMetersPerSecond); // why isn't this swapped for secondary motor?
        // this also doesn't use metres per second
    final double driveFeedforward = m_primaryFF.calculate(speedMetersPerSecond);
    // swerve has not a clue as to what speed it is going

    final double driveVoltage = driveFeedforward;//driveOutput + driveFeedforward;
   
    m_secondaryMotor.setVoltage(driveVoltage);
    // m_primaryMotor.setVoltage(driveVoltage);
    // copy sign
    // m_secondaryMotor.setVoltage(maxSpeedEnabled ? driveVoltage * (boostThrottle * (driveVoltage/driveVoltage)) : 0);
    // if(m_primaryEncoder.getVelocity() > )
    SmartDashboard.putNumber("drive voltage", driveVoltage);
    SmartDashboard.putNumber(m_name + " Primary velocity", m_secondaryEncoder.getVelocity());
    SmartDashboard.putNumber(m_name + " Secondary velocity", m_primaryEncoder.getVelocity());
    SmartDashboard.putNumber(m_name + " ecvt velocity", m_ecvt.getOutputSpeed());
    // if(Math.abs(m_secondaryEncoder.getVelocity()) > 3000){
    //   primarySpeedCounter++;
    //   if(primarySpeedCounter > 5){
    //     isPrimaryOverThreshold = true;
    //     RobotContainer.LED.set(0.05);
    //   }
    // }else{
    //   primarySpeedCounter += -1;
    //   if(primarySpeedCounter == 0){
    //     isPrimaryOverThreshold = false;
    //   }
    // }
    // if(!isPrimaryOverThreshold){
    //   maxSpeedEnabled = false;
    //   RobotContainer.LED.set(-0.15);
    // }
    // if(maxSpeedEnabled){
    //   RobotContainer.LED.set(0.07);
    // }
    double torqueMultiplier = MathUtil.applyDeadband(RobotContainer.m_driverController.getLeftTriggerAxis(), 0.1)*0.3; // [0,0.3]
    double back_drive = -(driveVoltage*torqueMultiplier);
    double forward_drive = driveVoltage * (boostThrottle * (driveVoltage/driveVoltage));
    // ^ why is this here, half of this is redundant (literally just Math.abs(driveVoltage) * boostThrottle.)
    
    if(back_drive > 5){
      back_drive = 5;
    }

    double output = 0;
    if(maxSpeedEnabled){
      // output = torqueRateLimiter.calculate(forward_drive);
      output = forward_drive;
      m_torqueRateLimiter.reset(0);
    }
    else{
      if(Math.abs(driveVoltage) < 3){
        output = 0;
      }else{
        output = m_torqueRateLimiter.calculate(back_drive);
      }
    }
    SmartDashboard.putNumber("Boost set voltage", output);
    SmartDashboard.putNumber("Boost motor rpm", m_primaryEncoder.getVelocity()); // rpm currently as factor is 1
    SmartDashboard.putNumber("Boost motor current", m_primaryMotor.getOutputCurrent());
    m_primaryMotor.setVoltage(output);
  }
  public double getSecondaryVelocity(){
    return m_secondaryEncoder.getVelocity();
  }

  /**
   * TODO: describe
   * 
   * @param speedMetersPerSecond
   * @param boostThrottle
   * @param maxSpeedEnabled
   */
  public void setAuto(double speedMetersPerSecond, double boostThrottle, boolean maxSpeedEnabled, boolean isLowTorqueModeEnabled) {
    // TODO: a bit of cleanup
    // Calculate the drive output from the drive PID controller.
    m_primaryPID.setP(Constants.Swerve.Gains.drivePID_kP_auto);
    m_primaryPID.setI(Constants.Swerve.Gains.drivePID_kI_auto);
    m_primaryPID.setD(Constants.Swerve.Gains.drivePID_kD_auto);
    final double driveOutput = m_primaryPID.calculate(m_secondaryEncoder.getVelocity(), m_ecvt.getRingSetpoint(speedMetersPerSecond));//m_primaryEncoder.getVelocity(), speedMetersPerSecond); // why isn't this swapped for secondary motor?
        // this also doesn't use metres per second
    final double driveFeedforward = m_primaryFF.calculate(speedMetersPerSecond);
    // swerve has not a clue as to what speed it is going
    double ff = 0;
    if(prevTime != -1){
      ff = Math.abs(prevVel - speedMetersPerSecond)/Math.abs(Timer.getFPGATimestamp()-prevTime)*1;
      // ff = Math.min(Math.abs(ff), Math.abs(driveVoltage*0.15));

    }
    final double driveVoltage = driveOutput + driveFeedforward;
    m_secondaryMotor.setVoltage(driveVoltage);
    // m_primaryMotor.setVoltage(driveVoltage);
    // copy sign
    // m_secondaryMotor.setVoltage(maxSpeedEnabled ? driveVoltage * (boostThrottle * (driveVoltage/driveVoltage)) : 0);
    // if(m_primaryEncoder.getVelocity() > )
    SmartDashboard.putNumber(m_name + " Primary velocity", m_primaryEncoder.getVelocity());
    SmartDashboard.putNumber(m_name+" Secondary velocity", m_secondaryEncoder.getVelocity());
    SmartDashboard.putNumber(m_name+" ecvt velocity", m_ecvt.getOutputSpeed());
    
    if(isLowTorqueModeEnabled){
    // m_primaryMotor.setVoltage(-ff);
    }
    else{
      m_primaryMotor.set(0);
    }
    
    prevVel = speedMetersPerSecond;
    prevTime = Timer.getFPGATimestamp();
    // m_primaryMotor.setVoltage(maxSpeedEnabled ? driveVoltage * (boostThrottle * (driveVoltage/driveVoltage)) : 0);
  }

  /** Update data being sent and recieved from NetworkTables. */
  public void updateNetworkTables() {
    SmartDashboard.putNumber(m_name + " Primary Motor Temp (C)", m_primaryMotor.getMotorTemperature());
    SmartDashboard.putNumber(m_name + " Secondary Motor Temp (C)", m_secondaryMotor.getMotorTemperature());
    SmartDashboard.putNumber(m_name + " Primary Motor Current (A)", m_primaryMotor.getOutputCurrent());
    SmartDashboard.putNumber(m_name + " Secondary Motor Current (A)", m_secondaryMotor.getOutputCurrent());
  }
}