// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.LED.*;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
// import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.util.Color;

/** Add your docs here. */
public class LED {
  // REV Blinkin pretends to be a PWM motor controller
  // private static final AddressableLED controller = new AddressableLED(2);
  private static Pattern pattern_drive;
  private static Pattern pattern_human;
  private static BlinkMode blinkMode_drive;
  private static BlinkMode blinkMode_human;
  private static AddressableLED led_dev;
  private static AddressableLEDBuffer led;
  private static int blinking_i;
  private static int blinking_i_human;

  //FRONT: ids 0-46 and 98-148
  // back 46-97
  int DRIVE_START = 46;
  int DRIVE_END = 97;
  final int HUMAN_START = 100;
  final int HUMAN_END = 200;
  private static LED m_instance = new LED();
  
  public LED(){
    pattern_drive = Pattern.SETUP;
    pattern_human = Pattern.SETUP;
    blinkMode_drive = BlinkMode.SOLID;
    blinkMode_human = BlinkMode.SOLID;
    led_dev = new AddressableLED(CONTROLLER_PWM_ID);
    led_dev.setLength(LED_LEN);
    led = new AddressableLEDBuffer(LED_LEN);
    setDrive(Pattern.SETUP, BlinkMode.SOLID);
    // setHuman(Pattern.CUBE, BlinkMode.SOLID);
    setBuffer();
    led_dev.setData(led);
    led_dev.start();
  }

  public static LED getInstance(){
    return m_instance;
  }

  public void periodic(){
    
    setBuffer();
    led_dev.setData(led);
  }

  public void setBuffer(){
    Color color = new Color(0, 0, 0);
    for(int i = DRIVE_START; i < DRIVE_END; i++){
      led.setLED(i, color);
    }
    switch(blinkMode_drive){
      case BLINKING_OFF:
        for(int i = DRIVE_START; i < DRIVE_END; i++){
          led.setLED(i, color);
        }
        if(blinking_i < 2){
          blinking_i++;
        }else{
          blinking_i = 0;
          blinkMode_drive = BlinkMode.BLINKING_ON;
        }
        break;
      case BLINKING_ON:
        color = PATTERN_MAP.getOrDefault(pattern_drive, new Color(0, 0, 0));
        for(int i = DRIVE_START; i < DRIVE_END; i++){
          led.setLED(i, color);
        }
        if(blinking_i < 2){
          blinking_i++;
        }else{
          blinking_i = 0;
          blinkMode_drive = BlinkMode.BLINKING_OFF;
        }
        break;
      case SOLID:
        color = PATTERN_MAP.getOrDefault(pattern_drive, new Color(0, 0, 0));
        for(int i = DRIVE_START; i < DRIVE_END; i++){
          led.setLED(i, color);
        }
        break;
      case OFF:
        color = new Color(0, 0, 0);
        for(int i = DRIVE_START; i < DRIVE_END; i++){
          led.setLED(i, color);
        }
        break;
      default:
        color = new Color(10, 10, 10);
        for(int i = DRIVE_START; i < DRIVE_END; i++){
          led.setLED(i, color);
        }
        DriverStation.reportError("INVALID LED STATE", null);
    }
    
    color = new Color(0, 0, 0);
    switch(blinkMode_human){
      case OFF:
        for(int i = 0; i < 46; i++){
          led.setLED(i, color);
        }for(int i = 98; i < 148; i++){
          led.setLED(i, color);
        }
        break;
      case SOLID:
        color = PATTERN_MAP.getOrDefault(pattern_human, new Color(0, 0, 0));

        for(int i = 0; i < 46; i++){
          led.setLED(i, color);
        }for(int i = 98; i < 148; i++){
          led.setLED(i, color);
        }
        break;
      case BLINKING_OFF:
        for(int i = 0; i < 46; i++){
          led.setLED(i, color);
        }for(int i = 98; i < 148; i++){
          led.setLED(i, color);
        }
        if(blinking_i_human < 2){
          blinking_i_human++;
        }else{
          blinking_i_human = 0;
          blinkMode_human = BlinkMode.BLINKING_ON;
        }
        break;
      case BLINKING_ON:
        color = PATTERN_MAP.getOrDefault(pattern_human, new Color(0, 0, 0));
        for(int i = 0; i < 46; i++){
          led.setLED(i, color);
        }for(int i = 98; i < 148; i++){
          led.setLED(i, color);
        }
        if(blinking_i_human < 2){
          blinking_i_human++;
        }else{
          blinking_i_human = 0;
          blinkMode_human = BlinkMode.BLINKING_OFF;
        }
        break;
    }
  }

  /** Colour values are located in {@link frc.robot.Constants.LED Constants::LED}. */
  public void setDrive(Pattern pattern, BlinkMode blink) {
    this.pattern_drive = pattern;
    this.blinkMode_drive = blink;
  }

  public void setHuman(Pattern pattern, BlinkMode blink) {
    this.pattern_human = pattern;
    this.blinkMode_human = blink;

    
    
    led_dev.setData(led);
  }

  public void setIdle() {
    this.setDrive(Pattern.SETUP, BlinkMode.SOLID);
  }
  public void setUrgentCone(){
    this.setHuman(Pattern.CONE, BlinkMode.BLINKING_ON);
  }
  public void setUrgentCube(){
    this.setHuman(Pattern.CUBE, BlinkMode.BLINKING_ON);
  }
}
