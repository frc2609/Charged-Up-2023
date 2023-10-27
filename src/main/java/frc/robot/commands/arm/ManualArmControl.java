// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.Xbox;
import frc.robot.subsystems.Arm;

/**
 * Adjust the arm setpoints proportional to the provided inputs.
 * When an input is not active (within input deadband), the setpoint is reset
 * to the current position of that part of the arm.
 * <p>This prevents the arm from moving when an input isn't being held, so
 * bumping a trigger or joystick won't cause the arm to rapidly accelerate.
 */
public class ManualArmControl extends CommandBase {
  /** Change in setpoint in degrees per second. */
  private static final double lowerAcceleration = 40.0;
  /** Change in setpoint in degrees per second. */
  private static final double upperAcceleration = 80.0;
  /** Change in setpoint in metres per second. */
  private static final double extensionAcceleration = 0.40;

  private final Arm arm;
  private final Timer timer = new Timer();
  private final Supplier<Double> lowerAdjustmentSupplier;
  private final Supplier<Double> upperAdjustmentSupplier;
  private final Supplier<Double> extensionExtendSupplier;
  private final Supplier<Double> extensionRetractSupplier;
  private double lowerSetpoint; // degrees
  private double upperSetpoint; // degrees
  private double extensionSetpoint;

  /** Creates a new ManualArmControl. */
  public ManualArmControl(Arm arm, Supplier<Double> lowerAdjustmentSupplier, Supplier<Double> upperAdjustmentSupplier, Supplier<Double> extensionExtendSupplier, Supplier<Double> extensionRetractSupplier) {
    this.arm = arm;
    this.lowerAdjustmentSupplier = lowerAdjustmentSupplier;
    this.upperAdjustmentSupplier = upperAdjustmentSupplier;
    this.extensionExtendSupplier = extensionExtendSupplier;
    this.extensionRetractSupplier = extensionRetractSupplier;

    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    lowerSetpoint = arm.getLowerAngle().getDegrees();
    upperSetpoint = arm.getUpperAngle().getDegrees();
    extensionSetpoint = arm.getExtensionDistance();
    timer.restart(); // 'reset()' will reset the time but won't actually start counting
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    final double delta = timer.get();
    final double lowerAdjustment = MathUtil.applyDeadband(lowerAdjustmentSupplier.get(), Xbox.JOYSTICK_DEADBAND);
    final double upperAdjustment = MathUtil.applyDeadband(upperAdjustmentSupplier.get(), Xbox.JOYSTICK_DEADBAND);
    final double extensionAdjustment = extensionExtendSupplier.get() - extensionRetractSupplier.get();

    lowerSetpoint += lowerAdjustment * lowerAcceleration * delta;
    upperSetpoint += upperAdjustment * upperAcceleration * delta;
    extensionSetpoint += extensionAdjustment * extensionAcceleration * delta;

    SmartDashboard.putNumber("delta", delta);
    SmartDashboard.putNumber("lower setpoint", lowerSetpoint);
    SmartDashboard.putNumber("upper setpoint", upperSetpoint);

    lowerSetpoint = lowerAdjustment == 0 ? arm.getLowerAngle().getDegrees() : lowerSetpoint;
    upperSetpoint = upperAdjustment == 0 ? arm.getUpperAngle().getDegrees() : upperSetpoint;
    extensionSetpoint = extensionAdjustment == 0 ? arm.getExtensionDistance() : extensionSetpoint;

    // TODO: clamp setpoints to max/min allowable values (so they don't just fly past the maximum)

    arm.setLowerAngle(Rotation2d.fromDegrees(lowerSetpoint));
    arm.setUpperAngle(Rotation2d.fromDegrees(upperSetpoint));
    arm.setExtensionLength(extensionSetpoint);

    // arm.openLoopset(MathUtil.applyDeadband(lowerAdjustmentSupplier.get(), Xbox.JOYSTICK_DEADBAND), MathUtil.applyDeadband(upperAdjustmentSupplier.get(), Xbox.JOYSTICK_DEADBAND), extensionExtendSupplier.get() - extensionRetractSupplier.get());

    // reset the timer so it tracks the time it takes to reach this again
    timer.reset();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    arm.holdPosition();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
