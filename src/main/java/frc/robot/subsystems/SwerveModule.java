// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.Swerve.*;
import static frc.robot.Constants.Swerve.Gains.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
//import edu.wpi.first.util.sendable.Sendable;
//import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Constants.Limits;

/**
 * Represents a single swerve drive module.
 */
public class SwerveModule { // implements Sendable {
  private final SwerveMotorGroup m_driveMotors;
  private final CANSparkMax m_rotationMotor;
  private final RelativeEncoder m_rotationEncoder;

  private final SparkMaxPIDController m_rotationPIDController;

  private final String m_name;

  // TODO: now using B to drive and A for boost, change documentation accordingly

  /** 
   * Creates a new SwerveModule.
   * 
   * @param name The module's name, added in front of all SmartDashboard values.
   * @param primaryMotorID The CAN ID of the primary drive motor (always engaged).
   * @param secondaryMotorID The CAN ID of the secondary drive motor (engaged for extra speed).
   * @param rotationMotorID The CAN ID of the rotation motor.
   * // TODO: this javadoc
   * @param invertDriveMotors name see SwerveMotorGroup//driveMotorsInverted Whether or not to invert both drive motors. The wheel should spin forward on positive inputs.
   * @param invertRotationMotor Whether or not to invert the rotation motor. The module should rotate counterclockwise on positive inputs.
   */
  public SwerveModule(
    String name,
    int primaryMotorID,
    int secondaryMotorID, // swap CANIDs plz
    int rotationMotorID,
    boolean invertDriveMotors,
    boolean invertRotationMotor
    )
  {
    m_driveMotors = new SwerveMotorGroup(primaryMotorID, secondaryMotorID, invertDriveMotors, name);
    m_rotationMotor = new CANSparkMax(rotationMotorID, MotorType.kBrushless);

    // TODO: that's not good. SwerveMotorGroup should handle this (or eliminate it if needed)
    // m_driveMotors.getEncoder().setPositionConversionFactor(DRIVE_POSITION_CONVERSION);
    // m_driveMotors.getEncoder().setVelocityConversionFactor(DRIVE_VELOCITY_CONVERSION);
    
    m_rotationEncoder = m_rotationMotor.getEncoder();
    m_rotationEncoder.setPositionConversionFactor(ROTATION_POSITION_CONVERSION);
    m_rotationEncoder.setVelocityConversionFactor(ROTATION_VELOCITY_CONVERSION);

    m_rotationPIDController = m_rotationMotor.getPIDController();
    configureSparkMaxPID();

    m_rotationMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    m_rotationMotor.setInverted(invertRotationMotor);
    m_rotationMotor.setSmartCurrentLimit(Limits.DRIVE_ROTATION_CURRENT);

    m_name = name;

    // SparkMaxPIDController and SimpleMotorFeedForward are not sent as they do not implement Sendable.
    // SendableRegistry.setName(m_drivePIDController, m_name, "Drive PID Controller");

    /* Send these values in the constructor so they appear in NetworkTables
     * before setDesiredState() is called for the first time. */
    SmartDashboard.putNumber(m_name + " Drive Setpoint (m/s)", 0);
    SmartDashboard.putNumber(m_name + " Angle Setpoint (rad)", 0);
    SmartDashboard.putNumber(m_name + " Drive Voltage", 0); // TODO: 99% sure not used.
  }

  public void simulateECVT(){
    m_driveMotors.simulateECVT();
  }
  public double getSecondaryVelocity(){
    return m_driveMotors.getSecondaryVelocity();
  }

  /** Update data being sent and recieved from NetworkTables. */
  public void updateNetworkTables() {
    SmartDashboard.putNumber(m_name + " Angle (rad)", m_rotationEncoder.getPosition());
    SmartDashboard.putNumber(m_name + " Angular Velocity (rad/s)", m_rotationEncoder.getVelocity());
    SmartDashboard.putNumber(m_name + " Distance Travelled (m)", m_driveMotors.getPosition());
    SmartDashboard.putNumber(m_name + " Velocity (m/s)", m_driveMotors.getVelocity());
    SmartDashboard.putNumber(m_name + " Rotation Motor Temp (C)", m_rotationMotor.getMotorTemperature());
    SmartDashboard.putNumber(m_name + " Rotation Motor Current (A)", m_rotationMotor.getOutputCurrent());
    m_driveMotors.updateNetworkTables();
  }

  /** Configure data being sent and recieved from NetworkTables. */
  // @Override
  // public void initSendable(SendableBuilder builder) {
  //   builder.addDoubleProperty("Distance Travelled (m)", m_driveEncoder::getPosition, null);
  //   builder.addDoubleProperty("Velocity (m/s)", m_driveEncoder::getVelocity, null);
  //   builder.addDoubleProperty("Angle (radians)", m_rotationEncoder::getPosition, null);
  // }

  /**
   * Set the constants for the rotation Spark Max's built in PID.
   */
  private void configureSparkMaxPID() {
    m_rotationPIDController.setP(rotationPID_kP);
    m_rotationPIDController.setI(rotationPID_kI);
    m_rotationPIDController.setD(rotationPID_kD);
    m_rotationPIDController.setIZone(rotationPID_IZone);
    m_rotationPIDController.setFF(rotationFF);
    m_rotationPIDController.setPositionPIDWrappingEnabled(true);
    m_rotationPIDController.setPositionPIDWrappingMinInput(-Math.PI);
    m_rotationPIDController.setPositionPIDWrappingMaxInput(Math.PI);
  }

  /**
   * Sends the module's drive PID constants to NetworkTables and updates them
   * whenever they are changed.
   * 
   * Only active while the function is called. Call this function periodically
   * to display and update the drive PID constants.
   * 
   * Calling this function while in a match is not recommended, as it will slow
   * down the robot code.
   */
  public void displayDrivePID() {
    // final double kP = SmartDashboard.getNumber(m_name + " Drive PID kP", m_drivePIDController.getP());
    // final double kI = SmartDashboard.getNumber(m_name + " Drive PID kI", m_drivePIDController.getI());
    // final double kD = SmartDashboard.getNumber(m_name + " Drive PID kD", m_drivePIDController.getD());
    // if (kP != m_drivePIDController.getP()) m_drivePIDController.setP(kP);
    // if (kI != m_drivePIDController.getI()) m_drivePIDController.setI(kI);
    // if (kD != m_drivePIDController.getD()) m_drivePIDController.setD(kD);
    // TODO: make it work for SwerveMotorGroup
  }

  /**
   * Sends the module's rotation feedforward constants to NetworkTables and
   * updates them whenever they are changed.
   * 
   * Only active while the function is called. Call this function periodically
   * to display and update the rotation feedforward constants.
   * 
   * Calling this function while in a match is not recommended, as it will slow
   * down the robot code.
   */
  public void displayRotationFF() {
    final double gain = SmartDashboard.getNumber(m_name + " Rotation FF Gain", m_rotationPIDController.getFF());
    if (gain != m_rotationPIDController.getFF()) m_rotationPIDController.setFF(gain);
  }

  /**
   * Sends the module's rotation PID constants to NetworkTables and updates the
   * PID's constants whenever they are changed.
   * 
   * Only active while the function is called. Call this function periodically
   * to display and update the rotation PID constants.
   * 
   * Calling this function while in a match is not recommended, as it will slow
   * down the robot code.
   */
  public void displayRotationPID() {
    final double kP = SmartDashboard.getNumber(m_name + " Rotation PID kP", m_rotationPIDController.getP());
    final double kI = SmartDashboard.getNumber(m_name + " Rotation PID kI", m_rotationPIDController.getI());
    final double kD = SmartDashboard.getNumber(m_name + " Rotation PID kD", m_rotationPIDController.getD());
    final double IZone = SmartDashboard.getNumber(m_name + " Rotation PID IZone",  m_rotationPIDController.getIZone());
    if (kP != m_rotationPIDController.getP()) m_rotationPIDController.setP(kP);
    if (kI != m_rotationPIDController.getI()) m_rotationPIDController.setI(kI);
    if (kD != m_rotationPIDController.getD()) m_rotationPIDController.setD(kD);
    if (IZone != m_rotationPIDController.getIZone()) m_rotationPIDController.setIZone(IZone);
    /* It is not necessary to send the values to NetworkTables because if the
     * values change from within the code, they will be overwritten by the
     * value stored in NetworkTables. If the values change because of a change
     * in NetworkTables, then NetworkTables will already have the value.
     */
  }

  /**
   * Returns the position of this module.
   * This is similar to `getState()`, but returns the position of the drive
   * motor instead of its velocity.
   * 
   * @return Position of the module
   */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
      m_driveMotors.getPosition(),
      new Rotation2d(m_rotationEncoder.getPosition())
    );
  }

  /**
   * Returns the angle of this module.
   * @return The angle of the swerve module in radians.
   */
  public double getRotationAngle() {
    return m_rotationEncoder.getPosition();
  }

  /**
   * Returns the current state of the module.
   * This is similar to `getPosition()`, but returns the velocity of the drive
   * motor instead of its position.
   * 
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(
      m_driveMotors.getVelocity(),
      new Rotation2d(m_rotationEncoder.getPosition())
    );
  }

  /**
   * Reset the rotation and drive encoder POSITIONS. DOES NOT HOME THE MODULE!
   * This should be used to reset the encoder position after manually homing
   * the module. (Does not reset encoder velocity.)
   */
  public void resetEncoders() {
    m_driveMotors.resetEncoders();
    m_rotationEncoder.setPosition(0);
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState, double secondaryThrottle, boolean maxSpeedEnabled) {
    /* If the robot is not being instructed to move, do not move any motors. 
     * This prevents the swerve module from returning to its original position
     * when the robot is not moving, which is the default behaviour of
     * ChassisSpeeds and SwerveModuleState.
     */
    
     m_rotationPIDController.setP(rotationPID_kP_auto);
     m_rotationPIDController.setI(rotationPID_kI_auto);
     m_rotationPIDController.setD(rotationPID_kD_auto);
     m_rotationPIDController.setFF(rotationFF_auto);
     
    // Temp
    m_rotationMotor.setIdleMode(IdleMode.kBrake);
    if (Math.abs(desiredState.speedMetersPerSecond) < MODULE_SPEED_DEADBAND) {
      stop();
      return;
    }
    
    // Optimize the desired state to avoid spinning further than 90 degrees
    SwerveModuleState optimizedState =
        SwerveModuleState.optimize(desiredState, new Rotation2d(m_rotationEncoder.getPosition()));

    SmartDashboard.putNumber(m_name + "Set M/S", optimizedState.angle.getRadians());
    m_driveMotors.set(optimizedState.speedMetersPerSecond, secondaryThrottle, maxSpeedEnabled);
    SmartDashboard.putNumber(m_name + "Set Angle", optimizedState.angle.getRadians());
    m_rotationPIDController.setReference(optimizedState.angle.getRadians(), ControlType.kPosition);
  }

  public void setDesiredStateAuto(SwerveModuleState desiredState, double secondaryThrottle, boolean maxSpeedEnabled) {
    /* If the robot is not being instructed to move, do not move any motors. 
     * This prevents the swerve module from returning to its original position
     * when the robot is not moving, which is the default behaviour of
     * ChassisSpeeds and SwerveModuleState.
     */
    m_rotationMotor.setIdleMode(IdleMode.kBrake);
    m_rotationPIDController.setP(rotationPID_kP_auto);
    m_rotationPIDController.setI(rotationPID_kI_auto);
    m_rotationPIDController.setD(rotationPID_kD_auto);
    m_rotationPIDController.setFF(rotationFF_auto);
    
    // Optimize the desired state to avoid spinning further than 90 degrees
    SwerveModuleState optimizedState =
        SwerveModuleState.optimize(desiredState, new Rotation2d(m_rotationEncoder.getPosition()));

    SmartDashboard.putNumber(m_name + "Set M/S", optimizedState.angle.getRadians());
    m_driveMotors.setAuto(optimizedState.speedMetersPerSecond, secondaryThrottle, maxSpeedEnabled);
    SmartDashboard.putNumber(m_name + "Set Angle", optimizedState.angle.getRadians());
    m_rotationPIDController.setReference(optimizedState.angle.getRadians(), ControlType.kPosition);
    
  }
  
  public void setRotationBrakeMode(boolean isBrake) {
    if (isBrake) {
      m_rotationMotor.setIdleMode(IdleMode.kBrake);
    } else {
      m_rotationMotor.setIdleMode(IdleMode.kCoast);
    }
  }

  /**
   * Rotate the module to the specified angle.
   * This method only has to be called once to set the rotation angle. In order
   * to have a useful return value, this function must be called periodically.
   * 
   * @param desiredAngle The desired angle of the module, in radians.
   * @return Whether or not the module has finished rotating.
   */
  public boolean rotateTo(double desiredAngle) {
    SmartDashboard.putNumber(m_name + " Angle Setpoint (rad)", desiredAngle);
    m_rotationPIDController.setReference(desiredAngle, ControlType.kPosition);
    return (m_rotationEncoder.getPosition() - desiredAngle) < ROTATION_ANGLE_TOLERANCE;
  }

  /**
   * Drive the module at the specified velocity.
   * This method must be called periodically in order to function.
   * 
   * @param desiredVelocity The desired velocity in m/s.
   */
  public void setVelocity(double desiredVelocity) {
    // TODO: this should work with new swerve
    // final double velocity = m_driveEncoder.getVelocity();
    // final double feedback = m_drivePIDController.calculate(velocity, desiredVelocity);
    // final double feedforward = m_driveFeedforward.calculate(desiredVelocity);
    // final double output = feedback+feedforward;
    // SmartDashboard.putNumber(m_name + " Drive Setpoint (m/s)", desiredVelocity);
    // SmartDashboard.putNumber(m_name + " Drive Voltage", output);
    // m_driveMotor.setVoltage(output);
  }

  /**
   * Stop all motors in this module.
   */
  public void stop() {
    m_driveMotors.set(0, 0, false);
    m_rotationMotor.setVoltage(0);
  }
}
