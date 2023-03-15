// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class WaitForInput extends CommandBase {
  private JoystickButton m_button;
  private Trigger m_Trigger;
  /** Finishes once given button is pressed. */
  public WaitForInput(JoystickButton button) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_button = button;
  }
  public WaitForInput(Trigger trigger) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_Trigger = trigger;
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
    if (m_button != null){
      return m_button.getAsBoolean();
    }
    return m_Trigger.getAsBoolean();
  }
}
