// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.HashMap;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
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

  public void createMap(){
    paths.put("LongThrowMid", new double[][]{{104.6,21.09000000000001,0.0},
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
    
    paths.put("ShortThrowMid", new double[][]{
    {104.6,21.09000000000001,0.0},
    {104.5767071759259,21.211072048611133,0.0},
    {104.50949074074073,21.5604513888889,0.0},
    {104.40234375,22.117382812500022,0.0},
    {104.25925925925925,22.86111111111112,0.0},
    {104.08423032407407,23.77088107638888,0.0},
    {103.88125000000001,24.825937500000013,0.0},
    {103.65431134259258,26.00552517361111,0.0},
    {103.40740740740739,27.288888888888916,0.0},
    {103.14453125,28.655273437500014,0.0},
    {102.86967592592592,30.083923611111118,0.0},
    {102.58683449074074,31.554084201388893,0.0},
    {102.3,33.04500000000001,0.0},
    {102.01316550925925,34.535915798611114,0.0},
    {101.73032407407406,36.00607638888887,0.0},
    {101.45546875000001,37.434726562499996,0.0},
    {101.19259259259259,38.8011111111111,0.0},
    {100.94568865740742,40.0844748263889,0.0},
    {100.71875,41.2640625,0.0},
    {100.51576967592592,42.31911892361111,0.0},
    {100.34074074074073,43.22888888888889,0.0},
    {100.19765625000001,43.972617187499985,0.0},
    {100.09050925925926,44.52954861111111,0.0},
    {100.02329282407408,44.8789279513889,0.0},
    {100.0,45.0,0.0},
    {100.0,45.0,0.0},
    {99.81770833333331,45.54181134259259,0.0},
    {99.29166666666667,47.10532407407408,0.0},
    {98.453125,49.59765625000001,0.0},
    {97.33333333333333,52.92592592592591,0.0},
    {95.96354166666667,56.99725115740739,0.0},
    {94.37500000000001,61.71875,0.0},
    {92.59895833333334,66.99754050925925,0.0},
    {90.66666666666667,72.74074074074076,0.0},
    {88.609375,78.85546875,0.0},
    {86.45833333333333,85.2488425925926,0.0},
    {84.24479166666669,91.82798032407406,0.0},
    {82.0,98.5,0.0},
    {79.75520833333333,105.17201967592592,0.0},
    {77.54166666666667,111.75115740740739,0.0},
    {75.390625,118.14453125,0.0},
    {73.33333333333333,124.25925925925928,0.0},
    {71.40104166666667,130.00245949074073,0.0},
    {69.625,135.28125,0.0},
    {68.03645833333333,140.00274884259255,0.0},
    {66.66666666666666,144.07407407407408,0.0},
    {65.546875,147.40234375,0.0},
    {64.70833333333333,149.8946759259259,0.0},
    {64.18229166666667,151.45818865740742,0.0},
    {64.0,152.0,0.0}});
    paths.put("PickToStow", new double[][]{{98.9,91.3,0.0},
    {99.0954732510288,88.89224965706447,0.0},
    {99.61934156378601,82.43947873799725,0.0},
    {100.37777777777777,73.09740740740742,0.0},
    {101.27695473251029,62.02175582990397,0.0},
    {102.22304526748972,50.36824417009601,0.0},
    {103.12222222222222,39.2925925925926,0.0},
    {103.88065843621399,29.950521262002766,0.0},
    {104.4045267489712,23.49775034293552,0.0}});
    paths.put("LongThrowPickup", new double[][]{{104.6,21.09000000000001,0.0},
    {103.88125000000001,31.075937500000006,0.0},
    {102.3,53.04500000000001,0.0},
    {100.71875,75.01406249999998,0.0},
    {100.0,84.99999999999999,0.0},
    {100.0,84.99999999999999,0.0},
    {99.45,88.14999999999999,0.0}});

    paths.put("LongThrowHigh", new double[][]{
      {104.6,21.09000000000001,0.0},
{104.5767071759259,21.211072048611133,0.0},
{104.50949074074073,21.5604513888889,0.0},
{104.40234375,22.117382812500022,0.0},
{104.25925925925925,22.86111111111112,0.0},
{104.08423032407407,23.77088107638888,0.0},
{103.88125000000001,24.825937500000013,0.0},
{103.65431134259258,26.00552517361111,0.0},
{103.40740740740739,27.288888888888916,0.0},
{103.14453125,28.655273437500014,0.0},
{102.86967592592592,30.083923611111118,0.0},
{102.58683449074074,31.554084201388893,0.0},
{102.3,33.04500000000001,0.0},
{102.01316550925925,34.535915798611114,0.0},
{101.73032407407406,36.00607638888887,0.0},
{101.45546875000001,37.434726562499996,0.0},
{101.19259259259259,38.8011111111111,0.0},
{100.94568865740742,40.0844748263889,0.0},
{100.71875,41.2640625,0.0},
{100.51576967592592,42.31911892361111,0.0},
{100.34074074074073,43.22888888888889,0.0},
{100.19765625000001,43.972617187499985,0.0},
{100.09050925925926,44.52954861111111,0.0},
{100.02329282407408,44.8789279513889,0.0},
{100.0,45.0,0.0},
{100.0,45.0,0.0},
{99.99442997685185,45.234447337962955,0.0},
{99.97835648148148,45.91099537037036,0.0},
{99.952734375,46.989453125,0.0},
{99.91851851851851,48.42962962962962,0.0},
{99.87666377314815,50.19133391203702,0.0},
{99.82812500000001,52.23437500000001,0.0},
{99.7738570601852,54.51856192129629,0.0},
{99.71481481481482,57.00370370370371,0.0},
{99.651953125,59.649609375000004,0.0},
{99.58622685185186,62.41608796296296,0.0},
{99.51859085648148,65.26294849537037,0.0},
{99.45,68.14999999999999,0.0},
{99.38140914351852,71.03705150462963,0.0},
{99.31377314814816,73.88391203703704,0.0},
{99.248046875,76.65039062499999,0.0},
{99.18518518518518,79.29629629629629,0.0},
{99.12614293981481,81.78143807870369,0.0},
{99.071875,84.065625,0.0},
{99.02333622685187,86.10866608796294,0.0},
{98.9814814814815,87.87037037037037,0.0},
{98.947265625,89.310546875,0.0},
{98.92164351851852,90.38900462962962,0.0},
{98.90557002314816,91.06555266203704,0.0},
{98.9,91.3,0.0},
{98.9,91.3,0.0},
{98.69542824074074,91.64280960648148,0.0022280092592592594},
{98.10509259259258,92.63206018518518,0.008657407407407407},
{97.16406250000001,94.208984375,0.01890625},
{95.90740740740742,96.31481481481482,0.03259259259259259},
{94.37019675925926,98.89078414351852,0.049334490740740734},
{92.5875,101.87812499999998,0.06875},
{90.59438657407408,105.21807002314814,0.0904571759259259},
{88.42592592592592,108.85185185185185,0.11407407407407408},
{86.1171875,112.72070312499999,0.13921875},
{83.70324074074075,116.76585648148149,0.16550925925925924},
{81.2191550925926,120.92854456018517,0.19256365740740738},
{78.7,125.15,0.22},
{76.18084490740742,129.3714554398148,0.24743634259259262},
{73.69675925925927,133.5341435185185,0.2744907407407407},
{71.2828125,137.579296875,0.30078125},
{68.97407407407408,141.44814814814816,0.32592592592592595},
{66.80561342592593,145.08192997685185,0.34954282407407405},
{64.8125,148.421875,0.37125},
{63.02980324074075,151.40921585648147,0.39066550925925914},
{61.49259259259259,153.98518518518517,0.4074074074074075},
{60.2359375,156.09101562499998,0.42109375},
{59.2949074074074,157.66793981481482,0.4313425925925925},
{58.704571759259245,158.6571903935185,0.4377719907407408},
{58.5,159.0,0.44}
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
    i=(int) Math.ceil(Timer.getFPGATimestamp()-startTime)*50; // 50 loops per second = 0.02 seconds per loop
    System.out.println("LOOP");
    System.out.println(i);

    if(i <= currentPath.length-1){
      m_armGripper.setLowerTargetAngle(currentPath[i][0]);
      m_armGripper.setUpperTargetAngle(currentPath[i][1]);
      m_armGripper.setExtensionTargetLength(currentPath[i][2]);
      
      // BeaverLogger.getInstance().logArm(currentPath[i], m_armGripper);
    }else{
      i = currentPath.length-1;
      m_armGripper.setLowerTargetAngle(currentPath[i][0]);
      m_armGripper.setUpperTargetAngle(currentPath[i][1]);
      m_armGripper.setExtensionTargetLength(currentPath[i][2]);
      // BeaverLogger.getInstance().logArm(currentPath[i], m_armGripper);
    }
    

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
      
      // BeaverLogger.getInstance().logArm(currentPath[currentPath.length-1], m_armGripper);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //return i>lowerSetpoint.length-1 // try this if it finishes too early
    boolean isLowerInTolerance = Math.abs(m_armGripper.getLowerArmAngleRelative()-currentPath[currentPath.length-1][0])<Tolerances.LOWER_ANGLE;
    boolean isUpperInTolerance = Math.abs(m_armGripper.getUpperArmAngleRelative()-currentPath[currentPath.length-1][1])<Tolerances.UPPER_ANGLE;
    boolean isExtensionInTolerance = Math.abs(m_armGripper.getExtensionDistance()-currentPath[currentPath.length-1][2])<Tolerances.EXTENSION_LENGTH;
    return i>=currentPath.length-1 && (isLowerInTolerance && isUpperInTolerance && isExtensionInTolerance);
  }
}
