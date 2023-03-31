// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDrive;

public class AlignToRotation extends CommandBase {
  private final PIDController m_anglePID = new PIDController(1, 0, 0);
  private final XboxController m_driverController;
  private final Rotation2d m_setpoint;
  private final SwerveDrive m_swerveDrive;

  /**
   * Creates a new AlignToRotation.
   * @param angle A Rotation2d of the desired angle.
   * @param swerveDrive The swerve drive subsystem.
   */
  public AlignToRotation(
    Rotation2d angle,
    SwerveDrive swerveDrive,
    XboxController driverController
  ) {
    m_driverController = driverController;
    m_setpoint = angle;
    m_swerveDrive = swerveDrive;
    addRequirements(swerveDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_anglePID.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // sketchy: doesn't square inputs
    m_swerveDrive.drive(
        -m_driverController.getLeftY(),
        -m_driverController.getLeftX(),
        m_anglePID.calculate(m_swerveDrive.getYaw().getRadians(), m_setpoint.getRadians()),
        true
    );
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_swerveDrive.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // move to constants
    return Math.abs(m_swerveDrive.getYaw().getDegrees() - m_setpoint.getDegrees()) <= 5.0;
  }
}
