// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.LED.*;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import frc.robot.Constants.PWMID;

/** Add your docs here. */
public class LED {
  // REV Blinkin pretends to be a PWM motor controller
  // private static final AddressableLED controller = new AddressableLED(2);
  private Pattern pattern_drive;
  private Pattern pattern_human;
  private BlinkMode blinkMode_drive;
  private BlinkMode blinkMode_human;
  public LED(){
    pattern_drive = Pattern.SETUP;
    pattern_human = Pattern.SETUP;
    blinkMode_drive = BlinkMode.SOLID;
    blinkMode_human = BlinkMode.SOLID;
  }
  public enum BlinkMode{
    SOLID,
    BLINKING_ON,
    BLINKING_OFF,
    OFF;
  };
  public enum Pattern{
    CONE,
    CUBE,
    FIRST_STAGE,
    SECOND_STAGE,
    BOOST,
    SETUP;
  };

  /** Colour values are located in {@link frc.robot.Constants.LED Constants::LED}. */
  public void setDrive(Pattern pattern, BlinkMode blink) {
    this.pattern_drive = pattern;
    this.blinkMode_drive = blink;
  }

  public void setHuman(Pattern pattern, BlinkMode blink) {
    this.pattern_human = pattern;
    this.blinkMode_human = blink;
  }

  public void setIdle() {
    this.setHuman(Pattern.SETUP, BlinkMode.SOLID);
  }
}
