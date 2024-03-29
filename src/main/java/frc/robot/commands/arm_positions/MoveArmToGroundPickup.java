package frc.robot.commands.arm_positions;

import static frc.robot.Constants.Arm.Position.*;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.arm.MoveArmToSetpoint;
import frc.robot.subsystems.Arm;

public class MoveArmToGroundPickup extends SequentialCommandGroup {

  /** Creates a new MoveArmToGroundPickup. */
  public MoveArmToGroundPickup(Arm arm) {
    addCommands(
        new MoveArmToSetpoint(GROUND_PICKUP_LOWER, GROUND_PICKUP_UPPER, GROUND_PICKUP_EXTENSION, false, false, false, arm)
    );
  }
}
