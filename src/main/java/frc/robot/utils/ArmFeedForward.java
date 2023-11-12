// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;

/** SimpleFeedForward with kg instead of ks */
public class ArmFeedForward extends SimpleMotorFeedforward {
  /**
   * Creates a new ArmFeedForward with the specified gains. Units of the gain values will
   * dictate units of the computed feedforward.
   *
   * @param kg The gravity gain.
   * @param kv The velocity gain.
   * @param ka The acceleration gain.
   */
  public ArmFeedForward(double kg, double kv, double ka) {
    super(kg, kv, ka);
  }

  /**
   * Creates a new ArmFeedForward with the specified gains. Units of the gain values will
   * dictate units of the computed feedforward.
   *
   * @param kg The gravity gain.
   * @param kv The velocity gain.
   */
  public ArmFeedForward(double kg, double kv) {
    super(kg, kv, 0);
  }

  /**
   * Calculates the feedforward from the gains and setpoints.
   *
   * @param gravitationalTorque The torque on the mechanism due to gravity.
   * @param velocity The velocity setpoint.
   * @param acceleration The acceleration setpoint.
   * @return The computed feedforward.
   */
  public double calculate(double gravitationalTorque, double velocity, double acceleration) {
    return ks * gravitationalTorque + kv * velocity + ka * acceleration;
  }
  
  /**
   * Calculates the feedforward from the gains and setpoints.
   *
   * @param gravitationalTorque The torque on the mechanism due to gravity.
   * @param velocity The velocity setpoint.
   * @return The computed feedforward.
   */
  public double calculate(double gravitationalTorque, double velocity) {
    return calculate(gravitationalTorque, velocity, 0);
  }
}
