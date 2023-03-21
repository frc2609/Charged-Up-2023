// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.SwerveDrive;

public class VisionAlign extends CommandBase {
  /** Creates a new VisionAlign. */
  SwerveDrive drive;
  XboxController driverController;
  PIDController rotationController, strafeController;
  NetworkTable limelight;
  public VisionAlign(SwerveDrive drive, XboxController driverController) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drive = drive;
    this.driverController = driverController;
    addRequirements(drive);
    SmartDashboard.putNumber("Rotation P", 0.01);
    SmartDashboard.putNumber("Strafe P", 0.15);
    rotationController = new PIDController(0.01, 0, 0);
    strafeController = new PIDController(0.15, 0, 0);
    limelight = NetworkTableInstance.getDefault().getTable("limelight");
    SmartDashboard.putNumber("gyroSetpVision", 180);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    rotationController.setSetpoint(180);
    strafeController.setSetpoint(0);
    limelight.getEntry("ledMode").setNumber(0);
    limelight.getEntry("camMode").setNumber(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    rotationController.setSetpoint(SmartDashboard.getNumber("gyroSetpVision", 0));
    rotationController.setP(SmartDashboard.getNumber("Rotation P", 0.01));
    strafeController.setP(SmartDashboard.getNumber("Strafe P", 0.01));
    SmartDashboard.putNumber("Gyro yaw", drive.getYaw());
    double rot = rotationController.calculate(drive.getYaw()%180);
    double strafe = strafeController.calculate(limelight.getEntry("tx").getDouble(0));
    if(rot < 0){
      rot = Math.max(-0.4, rot);
    }else{
      rot = Math.min(0.4, rot);
    }
    if(strafe < 0){
      strafe = Math.max(-0.5, strafe);
    }else{
      strafe = Math.min(0.5, strafe);
    }
    SmartDashboard.putNumber("starafePID", strafe);
    
    SmartDashboard.putNumber("rotPID", rot);
    // rot = rot*Math.signum(strafe);
    // invert forward/back b/c xbox is negative-upwards
    // invert strafe b/c strafe is left-positive, positive y axis moves right
    drive.drive(-driverController.getLeftY(), -strafe, rot, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.stop();
    limelight.getEntry("ledMode").setNumber(1);
    limelight.getEntry("camMode").setNumber(1);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
