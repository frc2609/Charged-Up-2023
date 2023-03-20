// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class WaitUntilTrigger extends CommandBase {
  /** Creates a new WaitUntilTrigger. */
  Trigger m_trigger;
  // TODO: Can be removed once we figure out how Commands.waitUntil works
  public WaitUntilTrigger(Trigger trigger) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_trigger = trigger;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_trigger.getAsBoolean();
  }
}
