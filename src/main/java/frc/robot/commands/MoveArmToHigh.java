package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ArmGripper;

public class MoveArmToHigh extends SequentialCommandGroup {

  /** Creates a new MoveArmToHigh. */
  public MoveArmToHigh(ArmGripper armGripper) {
    addCommands(
        // move arm away from ground
        new MoveArmToSetpoint(120.0, 90.0, 0.0, false, false, true, armGripper),
        new MoveArmToSetpoint(55, 165, 0.465, false, false, false, armGripper)
    );
  }
}
