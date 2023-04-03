// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.LED.*;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import frc.robot.Constants.PWMID;

/** Add your docs here. */
public class LED {
  // REV Blinkin pretends to be a PWM motor controller
  private static final Spark m_controller = new Spark(PWMID.REV_BLINKIN);
  private double currentColor;

  /** Colour values are located in {@link frc.robot.Constants.LED Constants::LED}. */
  public static void setColour(double colour) {
    m_controller.set(colour);
  }

  public static void setBlue() {
    setColour(BLUE);
  }

  public static void setCone() {
    setColour(YELLOW);
  }

  public static void setCube() {
    setColour(VIOLET);
  }

  public static void setGreen() {
    setColour(GREEN);
  }

  public static void setIdle() {
    if (DriverStation.getAlliance().equals(Alliance.Blue)) {
      setColour(BREATH_BLUE);
    } else {
      setColour(BREATH_RED);
    }
  }

  public static void setLime() {
    setColour(LIME);
  }

  public static void setRed() {
    setColour(RED);
  }

  public static void setUrgentCone() {
    setColour(C2_STROBE); // colour 2 = yellow
  }

  public static void setUrgentCube() {
    setColour(C1_STROBE); // colour 1 = purple
  }
}
