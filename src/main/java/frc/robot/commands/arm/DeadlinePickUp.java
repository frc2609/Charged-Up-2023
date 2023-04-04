// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;

// what does this file do?
public class DeadlinePickUp extends ParallelDeadlineGroup {
  /** Creates a new DeadlinePickUp. */
  public DeadlinePickUp() {
    super(Commands.waitSeconds(2));
  }
}
