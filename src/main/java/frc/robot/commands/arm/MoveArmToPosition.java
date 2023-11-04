// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class MoveArmToPosition extends CommandBase {
  private final Arm arm;
  private final double[][] path;
  private final boolean reverse;

  /** Creates a new MoveArmToPosition. */
  public MoveArmToPosition(Arm arm, double[][] path, boolean reverse) {
    this.arm = arm;
    this.path = path;
    this.reverse = reverse;    
    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    arm.currentPath = path;
    arm.isReverse = reverse;
    arm.startTime = Timer.getFPGATimestamp();
    arm.isEnabled = true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    arm.isEnabled = false;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // this is set to false when the path is completed
    return !arm.isEnabled;
  }

  // private double[][] fudgePath(double[][] path) {
  //   // so the fudge factor isn't applied to the original path
  //   double[][] fudgedPath = path.clone();
  //   for (int i = 0; i < fudgedPath.length; i++) {
  //     fudgedPath[i][1] = fudgedPath[i][1] - 90.0; // TODO: remove fudge factor
  //   }
  //   return fudgedPath;
  // }
}
