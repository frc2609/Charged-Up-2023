// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands.autonomous;

// import com.pathplanner.lib.PathPlanner;

// import com.pathplanner.lib.PathConstraints;

// import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
// import frc.robot.commands.MoveArmToSetpoint;
// import frc.robot.subsystems.ArmGripper;
// import frc.robot.subsystems.SwerveDrive;

// public class BalanceAndStow extends ParallelDeadlineGroup {
//   /** Creates a new BalanceAndStow. */
  
//   public BalanceAndStow(ArmGripper gripper, SwerveDrive drive) {
//     // Add the deadline command in the super() call. Add other commands using
//     // addCommands().
//     super(drive.generateAutoWithEvents(PathPlanner.loadPathGroup("ScoreThenAutobalance", new PathConstraints(1.5, 2.3))));
//     addCommands(new MoveArmToSetpoint(0.0, 0.0, 0, true, true, false, gripper));
//   }
// }
