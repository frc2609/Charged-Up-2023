// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.Arm.Tolerances;
import frc.robot.subsystems.ArmGripper;

public class MoveArmToMidProfiled extends CommandBase {
  double[] lowerSetpoint = {104.6, 104.55465357828778, 104.42968490717931, 104.24655879877662, 104.0269242740065, 103.78978299250845, 103.55113998390166, 103.32522138670555, 103.127014988728, 102.9769462107873, 102.91134083371442, 103.01240351436242, 103.5147767947738, 105.16870314197386, 107.64298677151268, 99.5, 95.5915565543023, 91.29231334981147, 86.98496903812922, 82.78338047361127, 78.71900090496733, 74.81267869566291, 71.08998129516213, 67.58978242301497, 64.36691515571066, 61.49337939483311, 59.058389249721486, 57.16721491694182, 55.93848378943022, 55.49992079122633};
  double[] upperSetpoint = {21.09000000000001, 22.123830386726777, 25.016554159876087, 29.45016131784148, 35.1064576501918, 41.66989630778309, 48.82792707149196, 56.2697786132945, 63.683917955878435, 70.75337448892111, 77.1452772420874, 82.4808763690195, 86.22898413817083, 87.34281289301157, 85.85701322848735, 93.99999999999999, 98.05421604044987, 102.76162251024677, 107.6957889793635, 112.6991268150185, 117.69645099007641, 122.62318136264618, 127.41001870483787, 131.97435751867602, 136.21763294924557, 140.02411331653718, 143.26085273278582, 145.77884922299987, 147.4157436158176, 148.00007920877368};
   double[] extensionSetpoint = {0.0, 0.0006009844616149289, 0.0022626095230138623, 0.004723708263959901, 0.007731197122829574, 0.011078111236603901, 0.014615964319668218, 0.01825659973612316, 0.021977623845941534, 0.025851804523267297, 0.030149257295463798, 0.03567892580214135, 0.04505340025326373, 0.06703167550621568, 0.09740550057528638, 0.0, 0.0, 0.0, 0.0, 0.0, 0.02669886484962961, 0.07913467122677215, 0.13953029048816704, 0.20423416108264447, 0.26955115609085845, 0.33171138941853595, 0.38685381895057425, 0.4310453287745308, 0.46035226694592823, 0.4709768933305409};
  double UpperError, UpperArm_P;
  double LowerError, LowerArm_P;
  ArmGripper m_armGripper;
  
  int i = 0;
  /** Moves arm to given setpoint. Finishes once within tolerance */
  public MoveArmToMidProfiled(ArmGripper armGripper) {
    m_armGripper = armGripper;
    addRequirements(armGripper);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    i=0;
    m_armGripper.setLowerTargetAngle(lowerSetpoint[0]);
    m_armGripper.setUpperTargetAngle(upperSetpoint[0]);
    m_armGripper.setExtensionTargetLength(extensionSetpoint[0]);
    i++;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    boolean isLowerInTolerance = Math.abs(m_armGripper.getLowerArmAngleRelative()-lowerSetpoint[i]) < 8; 
    boolean isUpperInTolerance = Math.abs(m_armGripper.getUpperArmAngleRelative()-upperSetpoint[i]) < 8; 
    boolean isExtensionInTolerance = Math.abs(m_armGripper.getExtensionDistance()-extensionSetpoint[i]) < Tolerances.EXTENSION_LENGTH;
    // if (isLowerInTolerance && isUpperInTolerance && isExtensionInTolerance){
    //   i++;
    // }
    i++;
    System.out.println(i);
    // i++;
    if(i <= lowerSetpoint.length-1){
      m_armGripper.setLowerTargetAngle(lowerSetpoint[i]);
      m_armGripper.setUpperTargetAngle(upperSetpoint[i]);
      m_armGripper.setExtensionTargetLength(extensionSetpoint[i]);
    }

    SmartDashboard.putNumber("lowerSetp", lowerSetpoint[i]);
    SmartDashboard.putNumber("upperSetp", upperSetpoint[i]);
    SmartDashboard.putNumber("extSetp", extensionSetpoint[i]);
    //SmartDashboard.putNumber("Lower Arm P", UpperArm_P);
    //SmartDashboard.putNumber("Upper Arm P", LowerArm_P);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
      System.out.println("PROFILED ARM TO MID FINISHED");
      m_armGripper.setLowerTargetAngle(lowerSetpoint[lowerSetpoint.length-1]);
      m_armGripper.setUpperTargetAngle(upperSetpoint[upperSetpoint.length-1]);
      m_armGripper.setExtensionTargetLength(extensionSetpoint[extensionSetpoint.length-1]);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //return i>lowerSetpoint.length-1 // try this if it finishes too early
    return i==lowerSetpoint.length-1;
  }
}
