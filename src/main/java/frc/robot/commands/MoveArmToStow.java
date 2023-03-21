package frc.robot.commands;

import static frc.robot.Constants.Arm.Position.*;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ArmGripper;

public class MoveArmToStow extends SequentialCommandGroup {

  /** Creates a new MoveArmToStow. */
  public MoveArmToStow(ArmGripper armGripper) {
    addCommands(
        // retract extension
        new MoveArmToSetpoint(0.0, 0.0, RETRACT_EXTENSION, true, true, false, armGripper),
        new MoveArmToSetpoint(STOW_LOWER, STOW_UPPER, STOW_EXTENSION, false, false, false, armGripper),
        new InstantCommand(armGripper::stopAllMotors, armGripper) // Prevent arm from gittering when driving around
    );
  }
}
