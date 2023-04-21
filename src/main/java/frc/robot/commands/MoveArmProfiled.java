// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.HashMap;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.Arm.Tolerances;
import frc.robot.commands.arm.ArmPaths;
import frc.robot.subsystems.ArmGripper;
// import frc.robot.utils.BeaverLogger;

public class MoveArmProfiled extends CommandBase {
  double[][] currentPath;
  ArmGripper m_armGripper;
  int i = 0;
  boolean isReverse = false;
  String currProfile;

  /** Moves arm to given setpoint. Finishes once within tolerance */
  public MoveArmProfiled(ArmGripper armGripper, String path, boolean isReversed) {
    m_armGripper = armGripper;
    addRequirements(armGripper);
    this.currProfile = path;
    this.currentPath = ArmPaths.paths.getOrDefault(path, new double[][]{{0.0,0.0,0.0},{0.0,0.0,0.0}});
    this.isReverse = isReversed;
    m_armGripper.isReverse = isReversed;
    m_armGripper.currentPath = this.currentPath;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_armGripper.currentPath = ArmPaths.paths.getOrDefault(currProfile, new double[][]{{0.0,0.0,0.0},{0.0,0.0,0.0}});; // makes sure the correct path is loaded
    m_armGripper.isMP = true;    // enables the loop
    m_armGripper.startTime = Timer.getFPGATimestamp();
    if(isReverse){
      m_armGripper.setLowerTargetAngle(currentPath[currentPath.length-1][0]);
      m_armGripper.setUpperTargetAngle(currentPath[currentPath.length-1][1]);
      m_armGripper.setExtensionTargetLength(currentPath[currentPath.length-1][2]);
    }else{
      m_armGripper.setLowerTargetAngle(currentPath[0][0]);
      m_armGripper.setUpperTargetAngle(currentPath[0][1]);
      m_armGripper.setExtensionTargetLength(currentPath[0][2]);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
      System.out.println("PROFILED ARM TO MID FINISHED");
      // if it gets interrupted, it cuts the loop
      m_armGripper.isMP = false;
      if(isReverse){
        m_armGripper.setLowerTargetAngle(currentPath[0][0]);
        m_armGripper.setUpperTargetAngle(currentPath[0][1]);
        m_armGripper.setExtensionTargetLength(currentPath[0][2]);
      }else{
        m_armGripper.setLowerTargetAngle(currentPath[currentPath.length-1][0]);
        m_armGripper.setUpperTargetAngle(currentPath[currentPath.length-1][1]);
        m_armGripper.setExtensionTargetLength(currentPath[currentPath.length-1][2]);
      }
      
      // BeaverLogger.getInstance().logArm(currentPath[currentPath.length-1], m_armGripper);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    
    boolean isLowerInTolerance = Math.abs(m_armGripper.getLowerAngleRelative()-currentPath[currentPath.length-1][0])<Tolerances.LOWER_ANGLE;
    boolean isUpperInTolerance = Math.abs(m_armGripper.getUpperAngleRelative()-currentPath[currentPath.length-1][1])<Tolerances.UPPER_ANGLE;
    boolean isExtensionInTolerance = Math.abs(m_armGripper.getExtensionDistance()-currentPath[currentPath.length-1][2])<Tolerances.EXTENSION_LENGTH;
    if(isReverse){
      isLowerInTolerance = Math.abs(m_armGripper.getLowerAngleRelative()-currentPath[0][0])<Tolerances.LOWER_ANGLE;
      isUpperInTolerance = Math.abs(m_armGripper.getUpperAngleRelative()-currentPath[0][1])<Tolerances.UPPER_ANGLE;
      isExtensionInTolerance = Math.abs(m_armGripper.getExtensionDistance()-currentPath[0][2])<Tolerances.EXTENSION_LENGTH;
    }
    return !m_armGripper.isMP && (isLowerInTolerance && isUpperInTolerance && isExtensionInTolerance);
  }
}
