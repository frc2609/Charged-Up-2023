// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import frc.robot.Constants.PWMID;

/** Add your docs here. */
public class LED {
  // add enum
  private static final Spark m_controller = new Spark(PWMID.REV_BLINKIN);

  public static void setColour(double colour) {
    m_controller.set(colour);
  }

  public static void setCone() {
    setColour(0.69);
  }

  public static void setCube() {
    setColour(0.91);
  }

  public static void setIdle() {
    setColour(0.87);
  }
}
