// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.Arm.Tolerances;
import frc.robot.subsystems.Arm;

public class MoveArmToSetpoint extends CommandBase {
  private static final int maxLoopsAtSetpoint = 3;
  private double lowerSetpoint, upperSetpoint, extensionSetpoint;
  private final boolean holdLower, holdUpper, holdExtension;
  private final Arm arm;
  private int loopsAtSetpoint = 0;

  /** Moves arm to given setpoint. Finishes once within tolerance. */
  public MoveArmToSetpoint(
      double lowerSetpoint,
      double upperSetpoint,
      double extensionSetpoint,
      boolean holdLower,
      boolean holdUpper,
      boolean holdSlider,
      Arm arm
  ) {
    this.holdLower = holdLower;
    this.holdUpper = holdUpper;
    this.holdExtension = holdSlider;
    this.lowerSetpoint = lowerSetpoint;
    this.upperSetpoint = upperSetpoint - 90.0; // TODO: remove fudge factor
    this.extensionSetpoint = extensionSetpoint;
    this.arm = arm;
    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    loopsAtSetpoint = 0;
    if (holdLower) {
      lowerSetpoint = arm.getLowerAngle().getDegrees();
    }
    if (holdUpper) {
      upperSetpoint = arm.getUpperAngle().getDegrees();
    }
    if (holdExtension) {
      extensionSetpoint = arm.getExtensionDistance();
    }
    arm.setLowerAngle(Rotation2d.fromDegrees(lowerSetpoint));
    arm.setUpperAngle(Rotation2d.fromDegrees(upperSetpoint));
    arm.setExtensionLength(extensionSetpoint);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    arm.holdPosition();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (isWithinTolerance()) loopsAtSetpoint++;
    else loopsAtSetpoint = 0; // reset if no longer at setpoint
    // wait until at setpoint for a certain amount of time
    return loopsAtSetpoint >= maxLoopsAtSetpoint;
  }

  private boolean isWithinTolerance() {
    double lowerError = arm.getLowerAngle().getDegrees() - lowerSetpoint;
    double upperError = arm.getUpperAngle().getDegrees() - upperSetpoint;
    double extensionError = arm.getExtensionDistance() - extensionSetpoint;
    boolean isLowerInTolerance = Math.abs(lowerError) < Tolerances.lowerAngle; 
    boolean isUpperInTolerance = Math.abs(upperError) < Tolerances.upperAngle;
    boolean isExtensionInTolerance = Math.abs(extensionError) < Tolerances.extensionLength;
    if (holdLower) {
      isLowerInTolerance = true;
    }
    if (holdUpper) {
      isUpperInTolerance = true;
    }
    if (holdExtension) {
      isExtensionInTolerance = true;
    }
    return isLowerInTolerance && isUpperInTolerance && isExtensionInTolerance;
  }
}
