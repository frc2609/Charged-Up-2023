// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.CloseGripper;
import frc.robot.commands.OpenGripper;
import frc.robot.subsystems.ArmGripper;
import frc.robot.subsystems.SwerveDrive;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoScoreConeThenBalance extends SequentialCommandGroup {
  /** Creates a new AutoScoreConeThenBalance. */
  List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup("ScoreThenAutobalance", new PathConstraints(1.7, 2.3));

  public AutoScoreConeThenBalance(ArmGripper gripper, SwerveDrive drive) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    // new AutoScoreConeHigh(gripper)
    addCommands(new SetOdometry(drive, pathGroup.get(0).getInitialPose()), new AutoScoreConeHigh(gripper),new OpenGripper(gripper),new BalanceAndStow(gripper, drive), new StopDrive(drive), new MoveArmToStow(gripper));
  }
  
}
