package frc.robot.commands.swerve;
// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands;

// import edu.wpi.first.math.MathUtil;
// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.wpilibj.XboxController;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.CommandBase;
// import frc.robot.Constants.Swerve;
// import frc.robot.Constants.Swerve.TeleopLimits;
// import frc.robot.subsystems.SwerveDrive;

// /**
//  * Bind this to {@code Trigger.whileTrue()} to use it (or a toggleOnTrue).
//  */
// public class AlignToRotation extends CommandBase {
//   // kP = rad/s per deg (yaw) of error
//   private final PIDController m_anglePID = new PIDController(0.1, 0, 0.004);
//   private final XboxController m_driverController;
//   private final Rotation2d m_setpoint;
//   private final SwerveDrive m_swerveDrive;

//   /**
//    * Creates a new AlignToRotation with the default tolerance.
//    * @param angle A Rotation2d of the desired angle.
//    * @param swerveDrive The swerve drive subsystem.
//    * @param driverController The driver's controller for translation control.
//    */
//   public AlignToRotation(
//     Rotation2d angle,
//     SwerveDrive swerveDrive,
//     XboxController driverController
//   ) {
//     m_driverController = driverController;
//     m_setpoint = angle;
//     m_swerveDrive = swerveDrive;
//     m_anglePID.enableContinuousInput(-180, 180);
//     m_anglePID.setTolerance(Swerve.DEFAULT_ROTATION_TOLERANCE.getDegrees());
//     addRequirements(swerveDrive);
//   }

//   /**
//    * Creates a new AlignToRotation.
//    * @param angle A Rotation2d of the desired angle.
//    * @param swerveDrive The swerve drive subsystem.
//    * @param driverController The driver's controller for translation control.
//    * @param tolerance How much error is acceptable for the desired angle.
//    */
//   public AlignToRotation(
//     Rotation2d angle,
//     SwerveDrive swerveDrive,
//     XboxController driverController,
//     Rotation2d tolerance
//   ) {
//     m_driverController = driverController;
//     m_setpoint = angle;
//     m_swerveDrive = swerveDrive;
//     m_anglePID.enableContinuousInput(-180, 180);
//     m_anglePID.setTolerance(tolerance.getDegrees());
//     addRequirements(swerveDrive);
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {
//     m_anglePID.reset();
//   }

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {
//     final double curAngle = m_swerveDrive.getYaw().getDegrees();
//     final double rotSpeed = MathUtil.clamp(m_anglePID.calculate(curAngle, m_setpoint.getDegrees()), -Math.PI / 2.0, Math.PI / 2.0);
//     m_swerveDrive.drive(
//         SwerveDrive.calculateManualInput(-m_driverController.getLeftY(), TeleopLimits.MAX_LINEAR_VELOCITY),
//         SwerveDrive.calculateManualInput(-m_driverController.getLeftX(), TeleopLimits.MAX_LINEAR_VELOCITY),
//         rotSpeed,
//         true
//     );
//     SmartDashboard.putNumber("alignRot/Target Yaw (deg, +ive = left)", m_setpoint.getDegrees());
//     SmartDashboard.putNumber("alignRot/Current Yaw (deg, +ive = left)", curAngle);
//     SmartDashboard.putNumber("alignRot/Calculated Rot Speed (radps [left])", rotSpeed);
//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {
//     m_swerveDrive.stop();
//   }

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     final boolean ended = m_anglePID.atSetpoint();
//     SmartDashboard.putBoolean("alignRot/atTarget", ended);
//     return ended;
//   }
// }
