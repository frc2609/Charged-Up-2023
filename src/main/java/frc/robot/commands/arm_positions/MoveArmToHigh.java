package frc.robot.commands.arm_positions;

import static frc.robot.Constants.Arm.Position.*;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.arm.MoveArmToSetpoint;
import frc.robot.subsystems.Arm;

public class MoveArmToHigh extends SequentialCommandGroup {

  /** Creates a new MoveArmToHigh. */
  public MoveArmToHigh(Arm arm) {
    addCommands(
        // move arm away from ground
        new MoveArmToSetpoint(RETRACT_LOWER, RETRACT_UPPER, RETRACT_EXTENSION, false, false, true, arm),
        new MoveArmToSetpoint(HIGH_LOWER, HIGH_UPPER, 0.0, false, false, true, arm),
        new MoveArmToSetpoint(0.0, 0.0, HIGH_EXTENSION, true, true, false, arm)
    );
  }
}
