package frc.robot.commands;

import static frc.robot.Constants.Arm.Position.*;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ArmGripper;

public class MoveArmToPickup extends SequentialCommandGroup {

  /** Creates a new MoveArmToPickup. */
  public MoveArmToPickup(ArmGripper armGripper) {
    addCommands(
        // move arm away from ground
        new MoveArmToSetpoint(RETRACT_LOWER, RETRACT_UPPER, RETRACT_EXTENSION, false, false, true, armGripper),
        new MoveArmToSetpoint(PICKUP_LOWER, PICKUP_UPPER, PICKUP_EXTENSION, false, false, false, armGripper)
    );
  }
}
