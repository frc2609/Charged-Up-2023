package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ArmGripper;

public class AdjustUpperSetpoint extends InstantCommand {
  private final double m_amount;
  private final ArmGripper m_armGripper;  

  public AdjustUpperSetpoint(ArmGripper armGripper, double amount) {
    m_amount = amount;
    m_armGripper = armGripper;
    addRequirements(armGripper);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    final double upperSetpoint = m_armGripper.getLowerArmAngleRelative() + m_amount;
    CommandScheduler.getInstance().schedule(new MoveArmToSetpoint(0.0, upperSetpoint, 0.0, true, false, true, m_armGripper));
  }
}