// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.Autonomous;
import frc.robot.Constants.Swerve.AutonomousLimits;
import frc.robot.subsystems.SwerveDrive;

public class ExampleAuto extends InstantCommand {
  PathPlannerTrajectory m_path = PathPlanner.loadPath(
      Autonomous.PATH_NAME,
      new PathConstraints(
          AutonomousLimits.MAX_LINEAR_VELOCITY,
          AutonomousLimits.MAX_LINEAR_ACCELERATION
      )
  );
  private final SwerveDrive m_swerveDrive;

  /** Creates a new ExampleAuto. */
  public ExampleAuto(SwerveDrive swerveDrive) {
    m_swerveDrive = swerveDrive;
    addRequirements(swerveDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_swerveDrive.generateFullAuto(m_path);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
