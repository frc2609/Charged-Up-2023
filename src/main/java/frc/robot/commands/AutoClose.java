// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmGripper;
import frc.robot.subsystems.LED;

public class AutoClose extends CommandBase {
  private final ArmGripper gripper;
  private final double threshold; // TODO: can remove, no longer necessary
  private int i; // name
  private int count = 2;

  /** Creates a new AutoClose. */
  public AutoClose(ArmGripper gripper, double threshold) {
    this.gripper = gripper;
    this.threshold = threshold;
  }
  public AutoClose(ArmGripper gripper, double threshold, int count) {
    this.gripper = gripper;
    this.threshold = threshold;
    this.count = count;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(gripper.getIntakeSensor()) {
      i++;
      LED.setWhite();
    } else {
      i = 0;
      LED.setIdle();
    }
    // TODO: move to a constant
    if (i >= count) {
      gripper.closeGripper();
      LED.setGreen();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return i >= count;
  }
}
