// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANID;
import frc.robot.Constants.DIO;
import frc.robot.Constants.Pneumatics;

public class Gripper extends SubsystemBase {
  private final Compressor compressor =
      new Compressor(CANID.PNEUMATICS_HUB, PneumaticsModuleType.REVPH);

  private final DoubleSolenoid gripperSolenoid = new DoubleSolenoid(
      CANID.PNEUMATICS_HUB,
      PneumaticsModuleType.REVPH, 
      Pneumatics.OPEN_SOLENOID_ID, // fwd
      Pneumatics.CLOSE_SOLENOID_ID // rev
  );

  private final DigitalInput intakeSensor = new DigitalInput(DIO.intakeSensor);

  /** Creates a new Gripper. */
  public Gripper() {
    compressor.enableDigital();
  }

  @Override
  public void periodic() {
    log();
  }

  private void log() {
    SmartDashboard.putBoolean("Intake Sensor", intakeSensor.get());
    if (gripperSolenoid.isFwdSolenoidDisabled()) {
      DataLogManager.log("OPEN SOLENOID DISABLED: CHECK FOR SHORTED/DISCONNECTED WIRES");
    }
    if (gripperSolenoid.isRevSolenoidDisabled()) {
      DataLogManager.log("CLOSE SOLENOID DISABLED: CHECK FOR SHORTED/DISCONNECTED WIRES");
    }
  }
  
  /**
   * Returns the output of the intake sensor.
   * @return Boolean true if there is a game piece in the gripper false otherwise.
   */
  public boolean getIntakeSensor() {
    return intakeSensor.get();
  }

  public void openGripper() {
    gripperSolenoid.set(kForward);
  }

  public void closeGripper() {
    gripperSolenoid.set(kReverse);
  }
}
