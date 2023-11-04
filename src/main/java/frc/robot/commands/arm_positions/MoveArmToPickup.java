package frc.robot.commands.arm_positions;

import static frc.robot.Constants.Arm.Position.*;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.arm.MoveArmToSetpoint;
import frc.robot.subsystems.Arm;

public class MoveArmToPickup extends SequentialCommandGroup {

  /** Creates a new MoveArmToPickup. */
  public MoveArmToPickup(Arm arm) {
    addCommands(
        // move arm away from ground
        new MoveArmToSetpoint(RETRACT_LOWER, RETRACT_UPPER, RETRACT_EXTENSION, false, false, true, arm),
        new MoveArmToSetpoint(PICKUP_LOWER, PICKUP_UPPER, PICKUP_EXTENSION, false, false, false, arm)
    );
  }
}
