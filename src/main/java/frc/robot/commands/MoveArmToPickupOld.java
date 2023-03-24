package frc.robot.commands;

import static frc.robot.Constants.Arm.Position.*;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ArmGripper;

public class MoveArmToPickupOld extends SequentialCommandGroup {

  /** Creates a new MoveArmToPickupOld. */
  public MoveArmToPickupOld(ArmGripper armGripper) {
    addCommands(
        // move arm away from ground
        new MoveArmToSetpoint(RETRACT_LOWER, RETRACT_UPPER, RETRACT_EXTENSION, false, false, true, armGripper),
        new MoveArmToSetpoint(93.1, 101.5, 0.0, false, false, false, armGripper)
    );
  }
}