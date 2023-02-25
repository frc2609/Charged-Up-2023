// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.SwerveDrive;

public class FollowAprilTag extends CommandBase {
  private final SwerveDrive m_swerveDrive;
  private final Limelight m_limelight;
  private final int m_aprilTagID;

  // private final PIDController m_xPID = new PIDController(1, 0, 0);
  private final PIDController m_yPID = new PIDController(0.1, 0, 0);
  // private final PIDController m_rotationPID = new PIDController(1, 0, 0);

  /** Creates a new FollowAprilTag. */
  public FollowAprilTag(SwerveDrive swerveDrive, Limelight limelight, int aprilTagID) {
    m_swerveDrive = swerveDrive;
    m_limelight = limelight;
    m_aprilTagID = aprilTagID;
    addRequirements(m_swerveDrive, m_limelight);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // This should be in a Limelight related file.
    // If the AprilTag is not visible, do not continue.
    if (m_limelight.tags.get(m_aprilTagID).get("tx") == null) {
      return;
    }
    final Object tx = m_limelight.tags.get(m_aprilTagID).get("tx");
    final double tx_double = (double)tx;
    SmartDashboard.putNumber("tx_double", tx_double);
    // final double xSpeed = m_xPID.calculate();
    final double ySpeed = m_yPID.calculate(tx_double, 0);
    // final double rotSpeed = m_rotationPID.calculate();

    m_swerveDrive.drive(0, ySpeed, 0, false);//ySpeed, rotSpeed, false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_swerveDrive.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
