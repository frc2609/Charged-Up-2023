package frc.robot.commands.arm_positions;

import static frc.robot.Constants.Arm.Position.*;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.arm.MoveArmToSetpoint;
import frc.robot.subsystems.Arm;

public class MoveArmToStow extends SequentialCommandGroup {

  /** Creates a new MoveArmToStow. */
  public MoveArmToStow(Arm arm) {
    addCommands(
        new MoveArmToSetpoint(0.0, 0.0, RETRACT_EXTENSION, true, true, false, arm),
        new MoveArmToSetpoint(STOW_LOWER, STOW_UPPER, STOW_EXTENSION, false, false, false, arm)
    );
  }
}
