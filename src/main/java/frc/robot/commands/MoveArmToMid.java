package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ArmGripper;

public class MoveArmToMid extends SequentialCommandGroup {

  /** Creates a new MoveArmToMid. */
  public MoveArmToMid(ArmGripper armGripper) {
    addCommands(
        // move arm away from ground
        new MoveArmToSetpoint(120.0, 90.0, 0.0, false, false, true, armGripper),
        new MoveArmToSetpoint(57.0, 152.0, 0.0, false, false, false, armGripper)
    );
  }
}
