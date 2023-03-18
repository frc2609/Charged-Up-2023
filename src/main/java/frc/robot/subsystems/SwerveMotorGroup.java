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
      return bevelToWheel*(carrierSpur/driveSpur)*((sunEncoder.getVelocity()*(sunTeeth/(sunTeeth+ringInner))+(ringEncoder.getVelocity()*(spurTeeth/ringOuter)*(ringInner/(sunTeeth+ringInner)))))*Constants.Swerve.DRIVE_VELOCITY_CONVERSION;
    }
    public double getSunSetpoint(double targetVelocity){
      return (targetVelocity-(ringEncoder.getVelocity()*(spurTeeth/ringOuter)*(ringInner/(sunTeeth+ringInner))*bevelToWheel*(carrierSpur/driveSpur)*Constants.Swerve.DRIVE_VELOCITY_CONVERSION))/Constants.Swerve.DRIVE_VELOCITY_CONVERSION;
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
    m_primaryMotor.setInverted(invertDriveMotors);
    m_secondaryMotor.setInverted(invertDriveMotors);
    m_primaryMotor.setIdleMode(IdleMode.kBrake);
    m_secondaryMotor.setIdleMode(IdleMode.kBrake);
    m_primaryMotor.setSmartCurrentLimit(Limits.DRIVE_PRIMARY_CURRENT);
    m_secondaryMotor.setSmartCurrentLimit(Limits.DRIVE_SECONDARY_CURRENT);
    m_ecvt = new ECVT(m_secondaryEncoder, m_primaryEncoder);
    m_name = name;
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
    // final double driveOutput =
        // m_primaryPID.calculate(m_secondaryEncoder.getVelocity(), speedMetersPerSecond);//m_primaryEncoder.getVelocity(), speedMetersPerSecond); // why isn't this swapped for secondary motor?
        // this also doesn't use metres per second
    final double driveFeedforward = m_primaryFF.calculate(speedMetersPerSecond);
    // swerve has not a clue as to what speed it is going

    final double driveVoltage = driveFeedforward;//driveOutput + driveFeedforward;
   
    m_secondaryMotor.setVoltage(driveVoltage);
    // m_primaryMotor.setVoltage(driveVoltage);
    // copy sign
    // m_secondaryMotor.setVoltage(maxSpeedEnabled ? driveVoltage * (secondaryThrottle * (driveVoltage/driveVoltage)) : 0);
    m_primaryMotor.setVoltage(maxSpeedEnabled ? driveVoltage * (secondaryThrottle * (driveVoltage/driveVoltage)) : 0);
  }

  /** Update data being sent and recieved from NetworkTables. */
  public void updateNetworkTables() {
    SmartDashboard.putNumber(m_name + " Primary Motor Temp (C°)", m_primaryMotor.getMotorTemperature());
    SmartDashboard.putNumber(m_name + " Secondary Motor Temp (C°)", m_secondaryMotor.getMotorTemperature());
    SmartDashboard.putNumber(m_name + " Primary Motor Current (A)", m_primaryMotor.getOutputCurrent());
    SmartDashboard.putNumber(m_name + " Secondary Motor Current (A)", m_secondaryMotor.getOutputCurrent());
  }
}