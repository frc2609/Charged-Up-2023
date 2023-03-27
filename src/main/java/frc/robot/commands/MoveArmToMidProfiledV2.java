// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.Arm.Tolerances;
import frc.robot.subsystems.ArmGripper;

public class MoveArmToMidProfiledV2 extends CommandBase {
  double[] lowerSetpoint = {104.6, 104.49332083232315, 104.2179444237321, 103.85880324236501, 103.48501908320381, 103.14845159945216, 102.90559322795109, 102.87062588732513, 103.44839220707667, 106.88732347099906, 99.5, 94.37942569325546, 88.32562880795152, 82.6616559001331, 77.54726928055831, 72.69620544784222, 69.12476597379043, 66.21748822073553, 64.29560649151173, 63.59710855800823};
  double[] upperSetpoint = {21.09000000000001, 23.52212498386341, 30.029696179834346, 39.41156712800532, 50.48240204162481, 62.07412727571932, 73.01403640167867, 82.07173350910834, 87.72616197673682, 86.61267652900095, 93.99999999999999, 99.4566511243028, 106.41113388066266, 113.37908484060767, 120.0394248209506, 126.51710045064885, 131.63449328546884, 135.84574909064997, 138.66831669093, 139.70289144199177};
  double[] extensionSetpoint = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  double UpperError, UpperArm_P;
  double LowerError, LowerArm_P;
  ArmGripper m_armGripper;
  boolean holdLower, holdUpper, holdSlider;
  
  int i = 0;
  /** Moves arm to given setpoint. Finishes once within tolerance */
  public MoveArmToMidProfiledV2(ArmGripper armGripper) {
    m_armGripper = armGripper;
    addRequirements(armGripper);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_armGripper.setLowerTargetAngle(lowerSetpoint[0]);
    m_armGripper.setUpperTargetAngle(upperSetpoint[0]);
    m_armGripper.setExtensionTargetLength(extensionSetpoint[0]);
    i++;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    boolean isLowerInTolerance = Math.abs(m_armGripper.getLowerArmAngleRelative()-lowerSetpoint[i]) < Tolerances.LOWER_ANGLE; 
    boolean isUpperInTolerance = Math.abs(m_armGripper.getUpperArmAngleRelative()-upperSetpoint[i]) < Tolerances.UPPER_ANGLE; 
    boolean isExtensionInTolerance = Math.abs(m_armGripper.getExtensionDistance()-upperSetpoint[i]) < Tolerances.EXTENSION_LENGTH;
    if(isLowerInTolerance && isUpperInTolerance && isExtensionInTolerance){
      i++;
    }
    // i++;
    m_armGripper.setLowerTargetAngle(lowerSetpoint[i]);
    m_armGripper.setUpperTargetAngle(upperSetpoint[i]);
    m_armGripper.setExtensionTargetLength(extensionSetpoint[i]);

    // SmartDashboard.putNumber("lowerError", Math.abs(m_armGripper.getLowerArmAngleRelative()-LOWER_SET));
    // SmartDashboard.putNumber("upperError", Math.abs(m_armGripper.getUpperArmAngleRelative()-UPPER_SET));
    // SmartDashboard.putNumber("extensionError", Math.abs(m_armGripper.getExtensionDistance()-EXTENSION_SET));
    //SmartDashboard.putNumber("Lower Arm P", UpperArm_P);
    //SmartDashboard.putNumber("Upper Arm P", LowerArm_P);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
      System.out.println("PROFILED ARM TO MID V2 FINISHED");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //return i>lowerSetpoint.length-1 // try this if it finishes too early
    return i==lowerSetpoint.length-1;
  }
}
