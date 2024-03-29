// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDrive;

/**
 * Autobalance the robot by driving forward/back until the robot's tilt is
 * within the specified angle tolerance.
 * <p>The robot must be facing towards the opposite driver stations (rotation = 0)
 * and it must already be on the charge station partially before calling this
 * command in order for it to work.
 */
public class Autobalance extends CommandBase {
  /** P gain for going up the ramp on the charge station. */
  private static final double START_P = 0.015;
  /** P gain for staying on the charge station. */
  // private static final double HOLD_P = 0.001;
  /** The acceptable amount of tilt error in degrees. */
  private static final double ANGLE_TOLERANCE = 8;
  /** The maximum autobalance speed in metres per second. */
  private static final double MAX_SPEED = 1.5;

  private final PIDController m_anglePIDController 
      = new PIDController(START_P, 0, 0.0015);
  private final SwerveDrive m_swerveDrive;

  /** 
   * Creates a new Autobalance.
   * @param swerveDrive The Swerve Drive subsystem.
   */
  public Autobalance(SwerveDrive swerveDrive) {
    m_anglePIDController.setTolerance(ANGLE_TOLERANCE);
    m_swerveDrive = swerveDrive;
    addRequirements(swerveDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // reset PID when command is rescheduled (or else starts at HOLD_P)
    m_anglePIDController.setP(START_P);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    /* 
     * The robot briefly hits the angle PID setpoint when the platform is
     * level, but continues moving because of inertia. Once it hits the
     * setpoint, the robot switches to a weaker PID to counteract its motion
     * and stay on the charge platform.
     */
    double roll = -m_swerveDrive.getGyro().getRoll();
    double output = m_anglePIDController.calculate(roll, 0);
    SmartDashboard.putNumber("Autobalance Output", output);
    double xSpeed = MathUtil.clamp(output, -MAX_SPEED, MAX_SPEED);
    SmartDashboard.putNumber("Autobalance Limited Output", xSpeed);
    if (m_anglePIDController.atSetpoint()) {
      m_swerveDrive.driveAuto(0, 0, 0, false);
    }else{
      m_swerveDrive.driveAuto(xSpeed, 0, 0, false);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_swerveDrive.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false; // this causes issues on flat ground
  }
}
