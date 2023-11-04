package frc.robot.commands.arm_positions;

// import static frc.robot.Constants.Arm.Position.*;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.arm.ArmPaths;
import frc.robot.commands.arm.MoveArmToPosition;
// import frc.robot.commands.arm.MoveArmToSetpoint;
import frc.robot.subsystems.Arm;

public class MoveArmToLow extends SequentialCommandGroup {

  /** Creates a new MoveArmToLow. */
  public MoveArmToLow(Arm arm) {
    addCommands(
        // move arm away from ground
        new MoveArmToPosition(arm, ArmPaths.stowToHybridNoExt, false)
        // new MoveArmToSetpoint(LOW_LOWER, LOW_UPPER, LOW_EXTENSION, false, false, false, arm)
    );
  }
}
