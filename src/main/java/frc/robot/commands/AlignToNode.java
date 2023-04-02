// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDrive;

// TODO: Attach the limelight and move cursor to correct position
// TODO: Make sure it tracks the correct target...

/**
 * Align the robot to a cone node using retroreflective tape.
 * Bind this to {@code Trigger.whileTrue()} to use it (or a toggleOnTrue).
 */
public class AlignToNode extends CommandBase {
  private final SwerveDrive m_swerveDrive;
  // I think these units are correct, may not be
  // kP = m/s per degree (limelight) of error
  private final PIDController m_xPID = new PIDController(0.01, 0, 0); // fwd/back (ty)
  private final PIDController m_yPID = new PIDController(0.01, 0, 0); // left/right (tx, yaw)
  // kP = rad/s per rad (yaw) of error
  private final PIDController m_rotPID = new PIDController(0.1, 0, 0); // rotation (yaw)
  private final NetworkTable m_limelight;

  /** Creates a new AlignToNode. */
  public AlignToNode(SwerveDrive swerveDrive) {
    m_swerveDrive = swerveDrive;
    m_limelight = NetworkTableInstance.getDefault().getTable("limelight");
    addRequirements(swerveDrive);
    // pids
    // can add a velocityTolerance as well if this doesn't work as expected
    m_xPID.setTolerance(3); // TODO: set
    m_yPID.setTolerance(3);
    m_rotPID.setTolerance(1.5);
    SmartDashboard.putNumber("align/tx", 0);
    SmartDashboard.putNumber("align/ty", 0);
    SmartDashboard.putNumber("align/rot", 0);
    SmartDashboard.putBoolean("align/atTarget", false);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_xPID.reset();
    m_yPID.reset();
    m_rotPID.reset();
    setVisionTracking(true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // distance to drive forward/back, ty = height
    // need to invert since pos will go fwd (relative to field) -> limelight up is positive
    final double curX = -SmartDashboard.getNumber("align/ty", 0);//m_limelight.getEntry("ty").getDouble(0);
    // distance to drive left/right, tx = dist left/right
    // need to invert since pos will go left (relative to field) -> limelight left is positive
    final double curY = -SmartDashboard.getNumber("align/tx", 0);//m_limelight.getEntry("tx").getDouble(0);
    // drive: pos = ccw = left, neg = cc = right, yaw also ccw positive so don't need to do anything
    final double curRot = SmartDashboard.getNumber("align/rot", 0);//m_swerveDrive.getYaw().getDegrees();
    /* Targets are all ZERO */
    final double xSpeed = m_xPID.calculate(curX, 0);
    final double ySpeed = m_yPID.calculate(curY, 0);
    final double rotSpeed = m_rotPID.calculate(curRot, 0);
    // These are INVERTED (except rotation)!
    // Make sure to take that into consideration when looking at these values.
    SmartDashboard.putNumber("align/Current X (target right and left)", curX);
    SmartDashboard.putNumber("align/Current Y (target down and up)", curY);
    SmartDashboard.putNumber("align/Current Yaw (pos = left)", curRot);
    SmartDashboard.putNumber("align/Calculated X Speed (mps [fwd])", xSpeed);
    SmartDashboard.putNumber("align/Calculated Y Speed (mps [left]", ySpeed);
    // TODO: is correct unit? (I'd hope so)
    SmartDashboard.putNumber("align/Calculated Rot Speed (radps [left])", rotSpeed);
    m_swerveDrive.drive(xSpeed, ySpeed, rotSpeed, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_swerveDrive.stop();
    setVisionTracking(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    final boolean ended = m_xPID.atSetpoint() && m_yPID.atSetpoint() && m_rotPID.atSetpoint();
    SmartDashboard.putBoolean("align/atTarget", ended);
    return false;
  }

  private void setVisionTracking(boolean isEnabled) {
    // 0 = on, 1 = off
    m_limelight.getEntry("ledMode").setNumber(isEnabled ? 0 : 1);
    m_limelight.getEntry("camMode").setNumber(isEnabled ? 0 : 1);
  }
}
