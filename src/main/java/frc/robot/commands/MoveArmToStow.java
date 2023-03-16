package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ArmGripper;

public class MoveArmToStow extends SequentialCommandGroup {

  /** Creates a new MoveArmToStop. */
  public MoveArmToStow(ArmGripper armGripper) {
    addCommands(
        new MoveArmToSetpoint(0.0, 0.0, 0.0, true, true, false, armGripper),
        new MoveArmToSetpoint(120, 25, 0.0, false, false, false, armGripper),
        new StopArm(armGripper) // Prevent arm from gittering when driving around
    );
  }
}
