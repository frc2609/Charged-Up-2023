package frc.robot.commands;

import static frc.robot.Constants.Arm.Position.*;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ArmGripper;

public class MoveArmToGroundPickup extends SequentialCommandGroup {

  /** Creates a new MoveArmToGroundPickup. */
  public MoveArmToGroundPickup(ArmGripper armGripper) {
    addCommands(
        // move arm away from ground
        new MoveArmToSetpoint(EXIT_STOW_LOWER, EXIT_STOW_UPPER, EXIT_STOW_EXTENSION, false, false, true, armGripper),
        new MoveArmToSetpoint(GROUND_PICKUP_LOWER, GROUND_PICKUP_UPPER, GROUND_PICKUP_EXTENSION, false, false, false, armGripper)
    );
  }
}
