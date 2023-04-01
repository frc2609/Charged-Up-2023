// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmGripper;
import frc.robot.subsystems.LED;

public class AutoClose extends CommandBase {
  /** Creates a new AutoClose. */
  ArmGripper gripper;
  LED led;
  int i;
  public AutoClose(ArmGripper gripper) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.gripper = gripper;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(gripper.isIntakeReadingValid() && gripper.getIntakeSensorDistance() < 105){
      i++;
      LED.setGreen();
    }else{
      i =0;
      LED.setIdle();
    }
    if(i>=5){
      gripper.closeGripper();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return i>=5;
  }
}
