package frc.robot.commands;

import static frc.robot.Constants.Arm.Position.*;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ArmGripper;

public class MoveArmToHigh extends SequentialCommandGroup {

  /** Creates a new MoveArmToHigh. */
  public MoveArmToHigh(ArmGripper armGripper) {
    addCommands(
        // move arm away from ground
        new MoveArmToSetpoint(RETRACT_LOWER, RETRACT_UPPER, RETRACT_EXTENSION, false, false, true, armGripper),
        new MoveArmToSetpoint(HIGH_LOWER, HIGH_UPPER, 0.0, false, false, true, armGripper),
        new MoveArmToSetpoint(0.0, 0.0, HIGH_EXTENSION, true, true, false, armGripper)
    );
  }
}
