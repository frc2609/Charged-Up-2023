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
  double UpperError, UpperArm_P;
  double LowerError, LowerArm_P;
  ArmGripper m_armGripper;
  int i = 0;
  double startTime;
  double jointErrorTolerance = Math.sqrt(50+Math.pow(3*Tolerances.EXTENSION_LENGTH,2)); // 5 Degrees each way
  boolean isReverse = false;
  String currProfile;

  

  public double[] getNearestSetpoint(double dt) {
    double predicted_lower = (m_armGripper.getLowerAngleRelative()+(dt*m_armGripper.getLowerJointAngularVelocity()));
    double predicted_upper = (m_armGripper.getUpperAngleRelative()+(dt*m_armGripper.getUpperJointAngularVelocity()));
    double predicted_extension = (m_armGripper.getExtensionDistance()+(dt*m_armGripper.getExtensionVelocity()));
    double curr_lowerError = Math.abs(predicted_lower-currentPath[i][0]);
    double curr_upperError = Math.abs(predicted_upper-currentPath[i][1]);
    double curr_extensionError = Math.abs(predicted_extension-currentPath[i][2]);
    // Note: i>0 always since we do i++ in initialize.
    double curr_to_prev_jointError = Math.sqrt(Math.pow(curr_lowerError,2)+Math.pow(curr_upperError,2)+Math.pow(curr_extensionError, 2));
    if(i+1<currentPath.length-1){
      double next_lowerError = Math.abs(predicted_lower-currentPath[i+1][0]); 
      double next_upperError = Math.abs(predicted_upper-currentPath[i+1][1]); 
      double next_extensionError = Math.abs(predicted_extension-currentPath[i+1][2]);
      double curr_to_next_jointError = Math.sqrt(Math.pow(next_lowerError,2)+Math.pow(next_upperError,2)+Math.pow(next_extensionError, 2));
      if(curr_to_next_jointError<curr_to_prev_jointError){
        i++;
        return getNearestSetpoint(dt);
      }
    }
    return currentPath[i];

  }
  /** Moves arm to given setpoint. Finishes once within tolerance */
  public MoveArmProfiled(ArmGripper armGripper, String path, boolean isReversed) {
    m_armGripper = armGripper;
    currentPath = ArmPaths.paths.getOrDefault(path, new double[][]{{0.0,0.0,0.0},{0.0,0.0,0.0}});
    addRequirements(armGripper);
    startTime = Timer.getFPGATimestamp();
    this.currProfile = path;
    this.isReverse = isReversed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    i=0;
    if(isReverse){
      m_armGripper.setLowerTargetAngle(currentPath[currentPath.length-1][0]);
      m_armGripper.setUpperTargetAngle(currentPath[currentPath.length-1][1]);
      m_armGripper.setExtensionTargetLength(currentPath[currentPath.length-1][2]);

    }else{
      m_armGripper.setLowerTargetAngle(currentPath[i][0]);
      m_armGripper.setUpperTargetAngle(currentPath[i][1]);
      m_armGripper.setExtensionTargetLength(currentPath[i][2]);

    }
    startTime = Timer.getFPGATimestamp();
  }
  public int getReverseIndex(int i){
    return (currentPath.length-1)-i;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // if (isLowerInTolerance && isUpperInTolerance && isExtensionInTolerance){
    //   i++;
    // }
    // if(currProfile == "ShortThrowMid" || currProfile == "LongThrowHighHD"){
    //   i++;
    // }else{
    //   getNearestSetpoint(Timer.getFPGATimestamp()-prevLoop);
    // }
    i=(int) Math.ceil(Timer.getFPGATimestamp()-startTime)*50; // 50 loops per second = 0.02 seconds per loop
    // System.out.println("LOOP");
    // System.out.println(i);

    if(i <= currentPath.length-1){
      if(isReverse){
        m_armGripper.setLowerTargetAngle(currentPath[getReverseIndex(i)][0]);
        m_armGripper.setUpperTargetAngle(currentPath[getReverseIndex(i)][1]);
        m_armGripper.setExtensionTargetLength(currentPath[getReverseIndex(i)][2]);
      }else{
        m_armGripper.setLowerTargetAngle(currentPath[i][0]);
        m_armGripper.setUpperTargetAngle(currentPath[i][1]);
        m_armGripper.setExtensionTargetLength(currentPath[i][2]);

      }
      
      // BeaverLogger.getInstance().logArm(currentPath[i], m_armGripper);
    }else{
      i = currentPath.length-1;
      if(isReverse){
        m_armGripper.setLowerTargetAngle(currentPath[0][0]);
        m_armGripper.setUpperTargetAngle(currentPath[0][1]);
        m_armGripper.setExtensionTargetLength(currentPath[0][2]);
      }else{
        m_armGripper.setLowerTargetAngle(currentPath[i][0]);
        m_armGripper.setUpperTargetAngle(currentPath[i][1]);
        m_armGripper.setExtensionTargetLength(currentPath[i][2]);
      }
      // BeaverLogger.getInstance().logArm(currentPath[i], m_armGripper);
    }
    

    // SmartDashboard.putNumber("lowerSetp", currentPath[i][0]);
    // SmartDashboard.putNumber("upperSetp", currentPath[i][1]);
    // SmartDashboard.putNumber("extSetp", currentPath[i][2]);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
      System.out.println("PROFILED ARM TO MID FINISHED");
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
    //return i>lowerSetpoint.length-1 // try this if it finishes too early
    
    boolean isLowerInTolerance = Math.abs(m_armGripper.getLowerAngleRelative()-currentPath[currentPath.length-1][0])<Tolerances.LOWER_ANGLE;
    boolean isUpperInTolerance = Math.abs(m_armGripper.getUpperAngleRelative()-currentPath[currentPath.length-1][1])<Tolerances.UPPER_ANGLE;
    boolean isExtensionInTolerance = Math.abs(m_armGripper.getExtensionDistance()-currentPath[currentPath.length-1][2])<Tolerances.EXTENSION_LENGTH;
    if(isReverse){
      isLowerInTolerance = Math.abs(m_armGripper.getLowerAngleRelative()-currentPath[0][0])<Tolerances.LOWER_ANGLE;
      isUpperInTolerance = Math.abs(m_armGripper.getUpperAngleRelative()-currentPath[0][1])<Tolerances.UPPER_ANGLE;
      isExtensionInTolerance = Math.abs(m_armGripper.getExtensionDistance()-currentPath[0][2])<Tolerances.EXTENSION_LENGTH;
    }
    return i>=currentPath.length-1 && (isLowerInTolerance && isUpperInTolerance && isExtensionInTolerance);
  }
}
