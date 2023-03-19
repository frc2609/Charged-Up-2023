package frc.robot.commands.autonomous;

import static frc.robot.Constants.Arm.Position.*;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.MoveArmToSetpoint;
import frc.robot.subsystems.ArmGripper;

public class MoveArmToHigh extends SequentialCommandGroup {

  /** Creates a new MoveArmToHigh. */
  public MoveArmToHigh(ArmGripper armGripper) {
    addCommands(
        // move arm away from ground
        new MoveArmToSetpoint(RETRACT_LOWER, RETRACT_UPPER, RETRACT_EXTENSION, false, false, true, armGripper),
        new MoveArmToSetpoint(HIGH_LOWER, HIGH_UPPER, HIGH_EXTENSION, false, false, false, armGripper)
    );
  }
}
