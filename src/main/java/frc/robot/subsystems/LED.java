// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.LED.*;

import java.sql.Driver;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.Constants.PWMID;

/** Add your docs here. */
public class LED {
  // REV Blinkin pretends to be a PWM motor controller
  // private static final AddressableLED controller = new AddressableLED(2);
  private Pattern pattern_drive;
  private Pattern pattern_human;
  private BlinkMode blinkMode_drive;
  private BlinkMode blinkMode_human;
  private AddressableLED led_dev;
  private AddressableLEDBuffer led;
  private int i;
  int DRIVE_START = 0;
  int DRIVE_END = 99;
  int HUMAN_START = 100;
  int HUMAN_END = 200;
  
  public LED(){
    pattern_drive = Pattern.SETUP;
    pattern_human = Pattern.SETUP;
    blinkMode_drive = BlinkMode.SOLID;
    blinkMode_human = BlinkMode.SOLID;
    led_dev = new AddressableLED(PWMID.LED);
    led_dev.setLength(LED_LEN);
    led = new AddressableLEDBuffer(LED_LEN);
    pattern_human = Pattern.SETUP;
    pattern_drive = Pattern.SETUP;
    blinkMode_drive = BlinkMode.SOLID;
    blinkMode_human = BlinkMode.SOLID;
    setDrive(Pattern.SETUP, BlinkMode.SOLID);
    setHuman(Pattern.SETUP, BlinkMode.SOLID);
    setBuffer();
    led_dev.setData(led);
    led_dev.start();
  }

  public void periodic(){
    setBuffer();
    led_dev.setData(led);

  }

  public void setBuffer(){
    Color color = new Color(0, 0, 0);
    switch(blinkMode_drive){
      case BLINKING_OFF:
        for(int i = HUMAN_START; i < HUMAN_END; i++){
          led.setLED(i, color);
        }
        if(i < 5){
          i++;
        }else{
          i = 0;
          blinkMode_drive = BlinkMode.BLINKING_ON;
        }
        break;
      case BLINKING_ON:
        color = PATTERN_MAP.getOrDefault(pattern_drive, new Color(0, 0, 0));
        for(int i = HUMAN_START; i < HUMAN_END; i++){
          led.setLED(i, color);
        }
        if(i < 5){
          i++;
        }else{
          i = 0;
          blinkMode_drive = BlinkMode.BLINKING_OFF;
        }
        break;
      case SOLID:
        color = PATTERN_MAP.getOrDefault(pattern_drive, new Color(0, 0, 0));
        color = new Color(0, 0, 0);
        for(int i = HUMAN_START; i < HUMAN_END; i++){
          led.setLED(i, color);
        }
        break;
      case OFF:
        color = new Color(0, 0, 0);
        for(int i = HUMAN_START; i < HUMAN_END; i++){
          led.setLED(i, color);
        }
        break;
      default:
        color = new Color(10, 10, 10);
        for(int i = HUMAN_START; i < HUMAN_END; i++){
          led.setLED(i, color);
        }
        DriverStation.reportError("INVALID LED STATE", null);
    }
    // TODO: Write a similar state machine for human
  }

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
