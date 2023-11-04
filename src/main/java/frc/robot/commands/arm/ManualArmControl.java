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
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
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
  private boolean lowerControlledLastLoop = false;
  private boolean upperControlledLastLoop = false;
  private boolean extensionControlledLastLoop = false;

  /** Creates a new ManualArmControl. */
  public ManualArmControl(Arm arm, Supplier<Double> lowerAdjustmentSupplier, Supplier<Double> upperAdjustmentSupplier, Supplier<Double> extensionExtendSupplier, Supplier<Double> extensionRetractSupplier) {
    this.arm = arm;
    this.lowerAdjustmentSupplier = lowerAdjustmentSupplier;
    this.upperAdjustmentSupplier = upperAdjustmentSupplier;
    this.extensionExtendSupplier = extensionExtendSupplier;
    this.extensionRetractSupplier = extensionRetractSupplier;

    addRequirements(arm);
  }
  
  public ManualArmControl(Arm arm, CommandXboxController controller) {
    this(
      arm,
      controller::getLeftY,
      () -> -controller.getRightY(),
      controller::getLeftTriggerAxis,
      controller::getRightTriggerAxis
    );
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

    // reset setpoint to current position when it is no longer being adjusted
    // only resets setpoint if the position was being controlled last loop to
    // prevent the setpoint from drifting if the arm moves
    if (lowerAdjustment != 0) {
      lowerControlledLastLoop = true;
    } else if (lowerControlledLastLoop) {
      lowerSetpoint = arm.getLowerAngle().getDegrees();
      lowerControlledLastLoop = false;
    }
    if (upperAdjustment != 0) {
      upperControlledLastLoop = true;
    } else if (upperControlledLastLoop) {
      upperSetpoint = arm.getUpperAngle().getDegrees();
      upperControlledLastLoop = false;
    }
    if (extensionAdjustment != 0) {
      extensionControlledLastLoop = true;
    } else if (extensionControlledLastLoop) {
      extensionSetpoint = arm.getExtensionDistance();
      extensionControlledLastLoop = false;
    }

    // TODO: clamp setpoints to max/min allowable values (so they don't just fly past the maximum)

    SmartDashboard.putNumber("manual/lower_setpoint", lowerSetpoint);
    SmartDashboard.putNumber("manual/upper_setpoint", upperSetpoint);
    SmartDashboard.putNumber("manual/extension_setpoint", extensionSetpoint);

    arm.setLowerAngle(Rotation2d.fromDegrees(lowerSetpoint));
    arm.setUpperAngle(Rotation2d.fromDegrees(upperSetpoint));
    arm.setExtensionLength(extensionSetpoint);

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
