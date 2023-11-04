// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
// import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.Arm.Tolerances;
import frc.robot.subsystems.Arm;

public class MoveArmToPosition extends CommandBase {
  private static final int maxLoopsAtSetpoint = 3;
  private int loopsAtSetpoint = 0;
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
    loopsAtSetpoint = 0;
    arm.currentPath = path;
    arm.isReverse = reverse;
    arm.startTime = Timer.getFPGATimestamp();
    arm.isEnabled = true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // // Called once the command ends or is interrupted.
  // @Override
  // public void end(boolean interrupted) {
  //   arm.isEnabled = false;
  //   if (interrupted) {
  //     arm.holdPosition();
  //   } else {
  //     // once the path is over, move to the last setpoint in the path
  //     double[] lastSetpoint = path[reverse ? 0 : path.length - 1];
  //     CommandScheduler.getInstance().schedule(new MoveArmToSetpoint(
  //       lastSetpoint[0], lastSetpoint[1] + 90.0, lastSetpoint[2], false, false, false, arm
  //                                       // TODO: remove fudge factor
  //     // TODO: use interrupt behaviour                                  // TODO: remove fudge factor
  //     ));
  //   }
  // }

  // // Returns true when the command should end.
  // @Override
  // public boolean isFinished() {
  //   // this is set to false when the path is completed
  //   return !arm.isEnabled;
  // }

  // private double[][] fudgePath(double[][] path) {
  //   // so the fudge factor isn't applied to the original path
  //   double[][] fudgedPath = path.clone();
  //   for (int i = 0; i < fudgedPath.length; i++) {
  //     fudgedPath[i][1] = fudgedPath[i][1] - 90.0; // TODO: remove fudge factor
  //   }
  //   return fudgedPath;
  // }

  @Override
  public void end(boolean interrupted) {
    arm.holdPosition();
    System.out.print("Ending move arm to position.");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (isWithinTolerance()) loopsAtSetpoint++;
    else loopsAtSetpoint = 0; // reset if no longer at setpoint
    // wait until at setpoint for a certain amount of time
    return loopsAtSetpoint >= maxLoopsAtSetpoint && !arm.isEnabled;
  }

  private boolean isWithinTolerance() {
    double[] finalSetpoint = path[reverse ? 0 : path.length - 1];
    double lowerError = arm.getLowerAngle().getDegrees() - finalSetpoint[0];
    double upperError = arm.getUpperAngle().getDegrees() - finalSetpoint[1];
    double extensionError = arm.getExtensionDistance() - finalSetpoint[2];
    boolean isLowerInTolerance = Math.abs(lowerError) < Tolerances.lowerAngle; 
    boolean isUpperInTolerance = Math.abs(upperError) < Tolerances.upperAngle;
    boolean isExtensionInTolerance = Math.abs(extensionError) < Tolerances.extensionLength;
    return isLowerInTolerance && isUpperInTolerance && isExtensionInTolerance;
  }
}
