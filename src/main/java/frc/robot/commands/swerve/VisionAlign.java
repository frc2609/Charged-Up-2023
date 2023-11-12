// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands.swerve;

// import edu.wpi.first.math.MathUtil;
// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.networktables.NetworkTable;
// import edu.wpi.first.networktables.NetworkTableInstance;
// import edu.wpi.first.wpilibj.XboxController;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.CommandBase;
// import frc.robot.subsystems.SwerveDrive;

// public class VisionAlign extends CommandBase {
//   private static final double ROT_MAX_OUTPUT = 0.4; // radians/s
//   private static final double STRAFE_MAX_OUTPUT = 0.5; // m/s

//   SwerveDrive drive;
//   XboxController driverController;
//   PIDController rotationController, strafeController;
//   NetworkTable limelight;

//   /** Creates a new VisionAlign. */
//   public VisionAlign(SwerveDrive drive, XboxController driverController) {
//     this.drive = drive;
//     this.driverController = driverController;
//     addRequirements(drive);
//     SmartDashboard.putNumber("Align to Node Rotation P", 0.01);
//     SmartDashboard.putNumber("Align to Node Translation P", 0.15);
//     rotationController = new PIDController(0.01, 0, 0);
//     strafeController = new PIDController(0.15, 0, 0);
//     limelight = NetworkTableInstance.getDefault().getTable("limelight");
//     SmartDashboard.putNumber("Align to Node Rotation Set", 180);
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {
//     rotationController.setSetpoint(180);
//     strafeController.setSetpoint(0);
//     // turn on LEDs and set limelight to vision tracking mode
//     limelight.getEntry("ledMode").setNumber(0);
//     limelight.getEntry("camMode").setNumber(0);
//   }

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {
//     rotationController.setSetpoint(SmartDashboard.getNumber("Align to Node Rotation Set", 0));
//     rotationController.setP(SmartDashboard.getNumber("Align to Node Rotation P", 0.01));
//     strafeController.setP(SmartDashboard.getNumber("Align to Node Strafe P", 0.01));
//     double rot = rotationController.calculate(drive.getYaw().getDegrees() % 180);
//     double strafe = strafeController.calculate(limelight.getEntry("tx").getDouble(0));
//     rot = MathUtil.clamp(rot, -ROT_MAX_OUTPUT, ROT_MAX_OUTPUT);
//     strafe = MathUtil.clamp(strafe, -STRAFE_MAX_OUTPUT, STRAFE_MAX_OUTPUT);
//     SmartDashboard.putNumber("Strafe PID Out (m/s)", strafe);
//     SmartDashboard.putNumber("Rot PID Out (rad/s)", rot);
//     // rot = rot*Math.signum(strafe);
//     // invert forward/back b/c xbox is negative-upwards
//     double adjustment = driverController == null ? 0.0 : -driverController.getLeftY();
//     // invert strafe b/c strafe is left-positive, positive y axis moves right
//     drive.drive(adjustment, -strafe, rot, true);
//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {
//     drive.stop();
//     // turn off LEDs and switch limelight to driver station camera mode
//     limelight.getEntry("ledMode").setNumber(1);
//     limelight.getEntry("camMode").setNumber(1);
//   }

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return false;
//   }
// }
