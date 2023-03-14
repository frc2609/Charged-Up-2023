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
import frc.robot.Constants;

// velocity conversion factors with/without motors


/** Add your docs here. */
public class SwerveMotorGroup {
  public class ECVT{
    private double ringRatio, sunRatio;
    private RelativeEncoder ringEncoder, sunEncoder;
    public ECVT(double ringRatio, double sunRatio, RelativeEncoder ringEncoder, RelativeEncoder sunEncoder){
      this.ringRatio = ringRatio;
      this.sunRatio = sunRatio;
      this.ringEncoder = ringEncoder;
      this.sunEncoder = sunEncoder;
    }
    public double getOutputSpeed(){
      return ((ringEncoder.getVelocity()*ringRatio)+(sunEncoder.getVelocity()*sunRatio))*Constants.Swerve.DRIVE_VELOCITY_CONVERSION;
    }
    public double getSunSetpoint(double targetVelocity){
      return (targetVelocity-(ringEncoder.getVelocity()*ringRatio*Constants.Swerve.DRIVE_VELOCITY_CONVERSION))/Constants.Swerve.DRIVE_VELOCITY_CONVERSION;
    }
  }

  private final CANSparkMax m_primaryMotor;
  private final CANSparkMax m_secondaryMotor;

  private final RelativeEncoder m_primaryEncoder;
  private final RelativeEncoder m_secondaryEncoder;
  private final ECVT m_ecvt;

  private final PIDController m_primaryPID =
      new PIDController(drivePID_kP, drivePID_kI, drivePID_kD);

  private final SimpleMotorFeedforward m_primaryFF =
      new SimpleMotorFeedforward(driveFF_kS, driveFF_kV, driveFF_kA);

  public SwerveMotorGroup(
    int primaryDriveMotorID,
    int secondaryDriveMotorID,
    boolean invertDriveMotors
  ) {
    m_primaryMotor = new CANSparkMax(primaryDriveMotorID, MotorType.kBrushless);
    m_secondaryMotor = new CANSparkMax(secondaryDriveMotorID, MotorType.kBrushless);
    m_primaryEncoder = m_primaryMotor.getEncoder();
    m_secondaryEncoder = m_secondaryMotor.getEncoder();
    m_primaryMotor.setInverted(invertDriveMotors);
    m_secondaryMotor.setInverted(invertDriveMotors);
    m_primaryMotor.setIdleMode(IdleMode.kBrake);
    m_secondaryMotor.setIdleMode(IdleMode.kBrake);
    m_ecvt = new ECVT(1/3, 1/3, m_secondaryEncoder, m_primaryEncoder);
  }

  public RelativeEncoder getEncoder() {
    return m_primaryEncoder;
  }

  // shouldn't exist
  public double getPosition() {
    // have fun
    // temp:
    return m_primaryEncoder.getPosition();
  }

  // shouldn't exist
  public double getVelocity() {
    // temp:
    return m_ecvt.getOutputSpeed();
  }

  public void set(double speedMetersPerSecond, double secondaryThrottle, boolean maxSpeedEnabled) {
    // Calculate the drive output from the drive PID controller.
    final double driveOutput =
        m_primaryPID.calculate(m_primaryEncoder.getVelocity(), speedMetersPerSecond);
    final double driveFeedforward = m_primaryFF.calculate(speedMetersPerSecond);

    final double driveVoltage = driveOutput + driveFeedforward;
   
    m_primaryMotor.setVoltage(driveVoltage);
    // copy sign
    m_secondaryMotor.setVoltage(maxSpeedEnabled ? driveVoltage * (secondaryThrottle * (driveVoltage/driveVoltage)) : 0);
  }
}