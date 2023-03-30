// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.HashMap;

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
  double startTime;
  double jointErrorTolerance = Math.sqrt(50+Math.pow(3*Tolerances.EXTENSION_LENGTH,2)); // 5 Degrees each way
  boolean isReverse = false;
  String currProfile;

  public void createMap() {
    paths.put("LongThrowMid", new double[][]{
        {104.6,21.09000000000001,0.0},
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
        {63.599444373251984,139.700555626748,0.0}
    });
    paths.put("ShortThrowMid", new double[][]{
        {104.6,21.09000000000001,0.0},
        {107.00625,24.044687500000006,0.0},
        {112.29999999999998,30.54500000000001,0.0},
        {117.59374999999999,37.04531249999999,0.0},
        {119.99999999999999,40.00000000000001,0.0},
        {119.99999999999999,40.00000000000001,0.0},
        {116.96875,48.4375,0.0},
        {110.3,67.0,0.0},
        {103.63125,85.5625,0.0},
        {100.60000000000001,93.99999999999999,0.0},
        {99.27626886145404,95.56721536351165,0.0},
        {95.72866941015089,99.7673525377229,0.0},
        {90.59259259259258,105.84814814814814,0.0},
        {84.50342935528121,113.05733882030177,0.0},
        {78.0965706447188,120.64266117969822,0.0},
        {72.0074074074074,127.85185185185183,0.0},
        {66.87133058984911,133.93264746227706,0.0},
        {63.323731138545945,138.13278463648834,0.0}
    });
    paths.put("PickToStow", new double[][]{
        {98.9,91.3,0.0},
        {99.0954732510288,88.89224965706447,0.0},
        {99.61934156378601,82.43947873799725,0.0},
        {100.37777777777777,73.09740740740742,0.0},
        {101.27695473251029,62.02175582990397,0.0},
        {102.22304526748972,50.36824417009601,0.0},
        {103.12222222222222,39.2925925925926,0.0},
        {103.88065843621399,29.950521262002766,0.0},
        {104.4045267489712,23.49775034293552,0.0}
    });
    paths.put("LongThrowPickup", new double[][]{
        {104.6,21.09000000000001,0.0},
        {103.88125000000001,31.075937500000006,0.0},
        {102.3,53.04500000000001,0.0},
        {100.71875,75.01406249999998,0.0},
        {100.0,84.99999999999999,0.0},
        {100.0,84.99999999999999,0.0},
        {99.45,88.14999999999999,0.0}
    });
    paths.put("LongThrowHigh", new double[][]{
      {104.6,21.09000000000001,0.0},
      {104.33936899862826,23.28170096021949,0.0},
      {103.64087791495197,29.155459533607715,0.0},
      {102.62962962962962,37.65925925925927,0.0},
      {101.43072702331962,47.741083676268865,0.0},
      {100.16927297668038,58.34891632373115,0.0},
      {98.97037037037036,68.43074074074075,0.0},
      {97.95912208504801,76.9345404663923,0.0},
      {97.26063100137175,82.80829903978052,0.0},
      {97.0,84.99999999999999,0.0},
      {95.67969821673523,87.43484224965707,0.015089163237311385},
      {92.14128943758573,93.96021947873798,0.05552812071330591},
      {87.01851851851852,103.40740740740739,0.11407407407407409},
      {80.94513031550068,114.6076817558299,0.1834842249657065},
      {74.55486968449932,126.3923182441701,0.25651577503429357},
      {68.48148148148148,137.59259259259258,0.3259259259259259},
      {63.35871056241426,147.03978052126203,0.38447187928669424},
      {59.820301783264746,153.56515775034293,0.4249108367626886},
      {58.5,156.0,0.44}
    });
    // extends earlier!
    paths.put("LongThrowHighHD", new double[][]{
      {104.6,21.09000000000001,0.0},
      {107.00625,22.482187500000013,0.0},
      {112.29999999999998,25.545000000000012,0.0},
      {117.59374999999999,28.60781249999999,0.0},
      {119.99999999999999,29.999999999999993,0.0},
      {119.99999999999999,29.999999999999993,0.0},
      {118.4375,33.125,0.03125},
      {115.0,40.00000000000001,0.1},
      {111.5625,46.874999999999986,0.16875},
      {110.0,49.99999999999999,0.2},
      {110.0,49.99999999999999,0.2},
      {108.23388203017832,53.63511659807956,0.20823045267489712},
      {103.50068587105625,63.377229080932786,0.23028806584362144},
      {96.64814814814815,77.48148148148147,0.26222222222222225},
      {88.52400548696845,94.20301783264746,0.300082304526749},
      {79.97599451303155,111.79698216735254,0.339917695473251},
      {71.85185185185185,128.5185185185185,0.37777777777777777},
      {64.99931412894375,142.62277091906725,0.4097119341563787},
      {60.26611796982167,152.36488340192045,0.43176954732510286},
      {58.5,156.0,0.44}
    });
  }

  public double[] getNearestSetpoint(double dt) {
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
    startTime = Timer.getFPGATimestamp();
    this.currProfile = path;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    i=0;
    m_armGripper.setLowerTargetAngle(currentPath[i][0]);
    m_armGripper.setUpperTargetAngle(currentPath[i][1]);
    m_armGripper.setExtensionTargetLength(currentPath[i][2]);
    startTime = Timer.getFPGATimestamp();
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
    i=(int) Math.ceil(startTime-Timer.getFPGATimestamp()*50); // 50 loops per second = 0.02 seconds per loop

    if(i <= currentPath.length-1){
      m_armGripper.setLowerTargetAngle(currentPath[i][0]);
      m_armGripper.setUpperTargetAngle(currentPath[i][1]);
      m_armGripper.setExtensionTargetLength(currentPath[i][2]);
    }
    
    // BeaverLogger.getInstance().logArm(currentPath[i], m_armGripper);

    SmartDashboard.putNumber("lowerSetp", currentPath[i][0]);
    SmartDashboard.putNumber("upperSetp", currentPath[i][1]);
    SmartDashboard.putNumber("extSetp", currentPath[i][2]);
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
    return i>=currentPath.length-1;
  }
}
