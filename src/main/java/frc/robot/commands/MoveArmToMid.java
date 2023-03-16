package frc.robot.commands;

import static frc.robot.Constants.Arm.Position.*;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ArmGripper;

public class MoveArmToMid extends SequentialCommandGroup {

  /** Creates a new MoveArmToMid. */
  public MoveArmToMid(ArmGripper armGripper) {
    addCommands(
        // move arm away from ground
        new MoveArmToSetpoint(RETRACT_LOWER, RETRACT_UPPER, RETRACT_EXTENSION, false, false, true, armGripper),
        new MoveArmToSetpoint(MID_LOWER, MID_UPPER, MID_EXTENSION, false, false, false, armGripper)
    );
  }
}
