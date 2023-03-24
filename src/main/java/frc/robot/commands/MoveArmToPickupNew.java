package frc.robot.commands;

import static frc.robot.Constants.Arm.Position.*;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ArmGripper;

public class MoveArmToPickupNew extends SequentialCommandGroup {

  /** Creates a new MoveArmToPickupNew. */
  public MoveArmToPickupNew(ArmGripper armGripper) {
    addCommands(
        // move arm away from ground
        new MoveArmToSetpoint(RETRACT_LOWER, RETRACT_UPPER, RETRACT_EXTENSION, false, false, true, armGripper),
        new MoveArmToSetpoint(98.9, 91.3, 0.07, false, false, false, armGripper)
    );
  }
}