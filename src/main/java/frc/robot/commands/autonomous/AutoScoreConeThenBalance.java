// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.MoveArmToStow;
import frc.robot.subsystems.ArmGripper;
import frc.robot.subsystems.SwerveDrive;

public class AutoScoreConeThenBalance extends SequentialCommandGroup {
  /** Creates a new AutoScoreConeThenBalance. */
  List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup(
      "ScoreThenAutobalance",
      new PathConstraints(1.7, 2.3)
  );

  public AutoScoreConeThenBalance(ArmGripper armGripper, SwerveDrive drive) {
    addCommands(
        new SetOdometry(drive, pathGroup.get(0).getInitialHolonomicPose()),
        new ScoreConeHigh(drive, armGripper),
        new InstantCommand(armGripper::openGripper),
        new BalanceAndStow(armGripper, drive),
        new InstantCommand(drive::stop, drive),
        new MoveArmToStow(armGripper)
    );
  }
}
