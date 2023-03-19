// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.Swerve.Gains.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.Limits;

// velocity conversion factors with/without motors
// Should this be in its own class?

/** Add your docs here. */
public class SwerveMotorGroup {
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

  private final CANSparkMax m_primaryMotor;
  private final CANSparkMax m_secondaryMotor;
  private boolean isPrimaryOverThreshold;
  private int primarySpeedCounter = 0;
  private final RelativeEncoder m_primaryEncoder;
  private final RelativeEncoder m_secondaryEncoder;
  private final ECVT m_ecvt;

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


  // shouldn't exist
  public double getPosition() {
    // have fun
    // temp:
    // return m_secondaryEncoder.getPosition();
    return m_ecvt.getPositionMeters();
  }
  public void resetEncoders(){
    m_primaryEncoder.setPosition(0);
    m_secondaryEncoder.setPosition(0);
  }

  // shouldn't exist
  public double getVelocity() {
    // temp:
    return m_ecvt.getOutputSpeed();
  }

  public void simulateECVT(){
    SmartDashboard.putNumber("Output SIM", m_ecvt.SIM_getOutputSpeed(SmartDashboard.getNumber("Test Primary RPM", 0), SmartDashboard.getNumber("Test Secondary RPM", 0)));
    SmartDashboard.putNumber("SunSetp SIM", m_ecvt.SIM_getSunSetpoint(SmartDashboard.getNumber("Test Target Vel", 0), SmartDashboard.getNumber("Test Secondary RPM", 0)));
    SmartDashboard.putNumber("position", m_ecvt.getPositionMeters());
  }

  public void set(double speedMetersPerSecond, double secondaryThrottle, boolean maxSpeedEnabled) {
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
    // m_secondaryMotor.setVoltage(maxSpeedEnabled ? driveVoltage * (secondaryThrottle * (driveVoltage/driveVoltage)) : 0);
    // if(m_primaryEncoder.getVelocity() > )
    SmartDashboard.putNumber("drive voltage", driveVoltage);
    SmartDashboard.putNumber("Primary velocity", m_secondaryEncoder.getVelocity());
    SmartDashboard.putNumber("Secondary velocity", m_primaryEncoder.getVelocity());
    SmartDashboard.putNumber("ecvt velocity", m_ecvt.getOutputSpeed());
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
    m_primaryMotor.setVoltage(maxSpeedEnabled ? driveVoltage * (secondaryThrottle * (driveVoltage/driveVoltage)) : 0);
  }
  public void setAuto(double speedMetersPerSecond, double secondaryThrottle, boolean maxSpeedEnabled) {
    // Calculate the drive output from the drive PID controller.
    m_primaryPID.setP(Constants.Swerve.Gains.drivePID_kP_auto);
    m_primaryPID.setI(Constants.Swerve.Gains.drivePID_kI_auto);
    m_primaryPID.setD(Constants.Swerve.Gains.drivePID_kD_auto);
    final double driveOutput =
    m_primaryPID.calculate(m_primaryEncoder.getVelocity(), m_ecvt.getSunSetpoint(speedMetersPerSecond));//m_primaryEncoder.getVelocity(), speedMetersPerSecond); // why isn't this swapped for secondary motor?
        // this also doesn't use metres per second
    final double driveFeedforward = m_primaryFF.calculate(speedMetersPerSecond);
    // swerve has not a clue as to what speed it is going

    final double driveVoltage = driveOutput + driveFeedforward;
    m_primaryMotor.setVoltage(driveVoltage);
    // m_primaryMotor.setVoltage(driveVoltage);
    // copy sign
    // m_secondaryMotor.setVoltage(maxSpeedEnabled ? driveVoltage * (secondaryThrottle * (driveVoltage/driveVoltage)) : 0);
    // if(m_primaryEncoder.getVelocity() > )
    SmartDashboard.putNumber("Primary velocity", m_secondaryEncoder.getVelocity());
    SmartDashboard.putNumber("Secondary velocity", m_primaryEncoder.getVelocity());
    SmartDashboard.putNumber("ecvt velocity", m_ecvt.getOutputSpeed());
    // m_primaryMotor.setVoltage(maxSpeedEnabled ? driveVoltage * (secondaryThrottle * (driveVoltage/driveVoltage)) : 0);
  }

  /** Update data being sent and recieved from NetworkTables. */
  public void updateNetworkTables() {
    SmartDashboard.putNumber(m_name + " Primary Motor Temp (C°)", m_primaryMotor.getMotorTemperature());
    SmartDashboard.putNumber(m_name + " Secondary Motor Temp (C°)", m_secondaryMotor.getMotorTemperature());
    SmartDashboard.putNumber(m_name + " Primary Motor Current (A)", m_primaryMotor.getOutputCurrent());
    SmartDashboard.putNumber(m_name + " Secondary Motor Current (A)", m_secondaryMotor.getOutputCurrent());
  }
}