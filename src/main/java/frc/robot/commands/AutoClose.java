// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmGripper;
import frc.robot.subsystems.LED;

public class AutoClose extends CommandBase {
  private final ArmGripper gripper;
  private int iterations;
  private int count = 2;

  /** Creates a new AutoClose.
   * @param gripper The ArmGripper to close automatically.
   * @param count How many loops the sensor must report a game piece for before closing.
   */
  public AutoClose(ArmGripper gripper, int count) {
    this.gripper = gripper;
    this.count = count;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(gripper.getIntakeSensor()) {
      iterations++;
      // LED.setWPhite();
    } else {
      iterations = 0;
      // LED.setIdle();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // only close the gripper if this ended successfully
    if (!interrupted) {
      gripper.closeGripper();
      // LED.setGreen();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return iterations >= count;
  }
}
