// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.Arm.Tolerances;
import frc.robot.subsystems.ArmGripper;
import frc.robot.utils.BeaverLogger;

public class MoveArmProfiled extends CommandBase {
  double[][] currentPath;
  
  double UpperError, UpperArm_P;
  double LowerError, LowerArm_P;
  ArmGripper m_armGripper;
  HashMap<String,double[][]> paths = new HashMap<String,double[][]>();
  int i = 0;
  double prevLoop;
  double jointErrorTolerance = Math.sqrt(50+Math.pow(3*Tolerances.EXTENSION_LENGTH,2)); // 5 Degrees each way

  public void createMap(){
    paths.put("StowIntMid", new double[][]{{104.6,21.09000000000001,0.0},
      {104.55465357828778,22.123830386726777,0.0},
      {104.42968490717931,25.016554159876087,0.0},
      {104.24655879877662,29.45016131784148,0.0},
      {104.0269242740065,35.1064576501918,0.0},
      {103.78978299250845,41.66989630778309,0.0},
      {103.55113998390166,48.82792707149196,0.0},
      {103.32522138670555,56.2697786132945,0.0},
      {103.127014988728,63.683917955878435,0.0},
      {102.9769462107873,70.75337448892111,0.0},
      {102.61134083371442,77.1452772420874,0.0},
      {101.81240351436242,82.4808763690195,0.0},
      {101.5147767947738,86.22898413817083,0.0},
      {101.16870314197386,87.34281289301157,0.0},
      {100.64298677151268,90.85701322848735,0.0},
      {99.5,93.99999999999999,0.0},
      {96.16269579954607,97.48016134331105,0.0},
      {92.1119805843657,101.93087655849142,0.0},
      {88.35620596272244,106.30093689442043,0.0},
      {84.67620776469111,110.76664937816604,0.0},
      {81.24662571216858,115.11051714497427,0.0},
      {78.03995458995445,119.31718826718841,0.0},
      {75.07650515594196,123.32349484405803,0.0},
      {72.37039801244245,127.07245913041471,0.0},
      {69.94613909373992,130.49671804911722,0.0},
      {67.83772299823622,133.51941985890667,0.0},
      {66.08942786865668,136.05342927420045,0.0},
      {64.75573571530187,138.00140714184099,0.0},
      {63.90103108877357,139.2561117683693,0.0},
      {63.599444373251984,139.700555626748,0.0}});
    
  }
  public double[] getNearestSetpoint(double dt){
    double predicted_lower = (m_armGripper.getLowerArmAngleRelative()+(dt*m_armGripper.getLowerJointAngularVelocity()));
    double predicted_upper = (m_armGripper.getUpperArmAngleRelative()+(dt*m_armGripper.getUpperJointAngularVelocity()));
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
  public MoveArmProfiled(ArmGripper armGripper, String path) {
    createMap();
    m_armGripper = armGripper;
    currentPath = paths.getOrDefault(path, new double[][]{{0.0,0.0,0.0},{0.0,0.0,0.0}});
    addRequirements(armGripper);
    prevLoop = Timer.getFPGATimestamp();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    i=0;
    m_armGripper.setLowerTargetAngle(currentPath[i][0]);
    m_armGripper.setUpperTargetAngle(currentPath[i][1]);
    // m_armGripper.setExtensionTargetLength(currentPath[i][2]);
    prevLoop = Timer.getFPGATimestamp();
    i++;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // if (isLowerInTolerance && isUpperInTolerance && isExtensionInTolerance){
    //   i++;
    // }
    // i++;
    getNearestSetpoint(Timer.getFPGATimestamp()-prevLoop);
    // i++;
    System.out.println(i);
    // i++;

    if(i <= currentPath.length-1){
      m_armGripper.setLowerTargetAngle(currentPath[i][0]);
      m_armGripper.setUpperTargetAngle(currentPath[i][1]);
      // m_armGripper.setExtensionTargetLength(currentPath[i][2]);
    }
    
    BeaverLogger.getInstance().logArm(currentPath[i], m_armGripper);

    SmartDashboard.putNumber("lowerSetp", currentPath[i][0]);
    SmartDashboard.putNumber("upperSetp", currentPath[i][1]);
    SmartDashboard.putNumber("extSetp", currentPath[i][2]);
    //SmartDashboard.putNumber("Lower Arm P", UpperArm_P);
    //SmartDashboard.putNumber("Upper Arm P", LowerArm_P);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
      System.out.println("PROFILED ARM TO MID FINISHED");
      m_armGripper.setLowerTargetAngle(currentPath[currentPath.length-1][0]);
      m_armGripper.setUpperTargetAngle(currentPath[currentPath.length-1][1]);
      m_armGripper.setExtensionTargetLength(currentPath[currentPath.length-1][2]);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //return i>lowerSetpoint.length-1 // try this if it finishes too early
    return i==currentPath.length-1;
  }
}
