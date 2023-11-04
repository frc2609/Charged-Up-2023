// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swerve;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDrive;

/** This doesn't particularly work so I'd recommend not using it.  */
public class ResetModules extends CommandBase {
  private final SwerveDrive drive;
  private final double angle;
  private boolean isReset;

  /** Creates a new ResetModules. */
  public ResetModules(SwerveDrive drive, double angle) {
    this.drive = drive;
    this.angle = angle;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    isReset = drive.setRotationAngle(angle);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("Swerve Modules Reset Successfully.");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isReset;
  }
}
