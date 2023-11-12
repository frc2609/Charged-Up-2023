// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

/**
 * Set the arm setpoints to a value from NetworkTables.
 * Allows easier tuning of the arm using a number slider on Shuffleboard.
 */
public class ManualDashboardArmControl extends CommandBase {
  private final Arm arm;

  /** Creates a new ManualDashboardArmControl. */
  public ManualDashboardArmControl(Arm arm) {
    this.arm = arm;
    SmartDashboard.putNumber("manual_dashboard/lower_setpoint", 0);
    SmartDashboard.putNumber("manual_dashboard/upper_setpoint", 0); 
    SmartDashboard.putNumber("manual_dashboard/extension_setpoint", 0);

    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putNumber("manual_dashboard/lower_setpoint", arm.getLowerAngle().getDegrees());
    SmartDashboard.putNumber("manual_dashboard/upper_setpoint", arm.getUpperAngle().getDegrees());
    SmartDashboard.putNumber("manual_dashboard/extension_setpoint", arm.getExtensionDistance());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // TODO: clamp to allowable values
    arm.setLowerAngle(Rotation2d.fromDegrees(SmartDashboard.getNumber("manual_dashboard/lower_setpoint", arm.getLowerAngle().getDegrees())));
    arm.setUpperAngle(Rotation2d.fromDegrees(SmartDashboard.getNumber("manual_dashboard/upper_setpoint", arm.getUpperAngle().getDegrees())));
    arm.setExtensionLength(SmartDashboard.getNumber("manual_dashboard/extension_setpoint", arm.getUpperAngle().getDegrees()));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    arm.holdPosition();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
