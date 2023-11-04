package frc.robot.commands.arm_positions;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.arm.ArmPaths;
import frc.robot.commands.arm.MoveArmToPosition;
import frc.robot.subsystems.Arm;

public class MoveArmToLow extends SequentialCommandGroup {

  /** Creates a new MoveArmToLow. */
  public MoveArmToLow(Arm arm) {
    addCommands(
        new MoveArmToPosition(arm, ArmPaths.stowToLow, false)
    );
  }
}
