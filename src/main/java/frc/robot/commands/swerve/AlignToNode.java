package frc.robot.commands.swerve;
// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands;

// import edu.wpi.first.math.MathUtil;
// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.networktables.NetworkTable;
// import edu.wpi.first.networktables.NetworkTableInstance;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.CommandBase;
// import frc.robot.subsystems.SwerveDrive;

// /**
//  * Align the robot to a cone node using retroreflective tape.
//  * Bind this to {@code Trigger.whileTrue()} to use it (or a toggleOnTrue).
//  */
// public class AlignToNode extends CommandBase {
//   private final SwerveDrive m_swerveDrive;
//   // kP = m/s per degree (limelight) of error
//   private final PIDController m_xPID = new PIDController(0.1, 0, 0); // fwd/back (ty) note: should be larger than y as this is typically smaller
//   private final PIDController m_yPID = new PIDController(0.05, 0, 0); // left/right (tx, yaw)
//   // kP = rad/s per deg (yaw) of error
//   private final PIDController m_rotPID = new PIDController(0.1, 0, 0.004); // rotation (yaw)
//   private static final NetworkTable m_limelight = NetworkTableInstance.getDefault().getTable("limelight");

//   /** Creates a new AlignToNode. */
//   public AlignToNode(SwerveDrive swerveDrive) {
//     m_swerveDrive = swerveDrive;
//     addRequirements(swerveDrive);
//     // pids
//     // can add a velocityTolerance as well if this doesn't work as expected
//     m_xPID.setTolerance(0.05);
//     m_yPID.setTolerance(0.05);
//     m_rotPID.setTolerance(1.5);
//     m_rotPID.enableContinuousInput(-180, 180);
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {
//     // TODO: set correct pipeline and setup pipeline settings
//     m_xPID.reset();
//     m_yPID.reset();
//     m_rotPID.reset();
//     setVisionTracking(true);
//   }

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {
//     // distance to drive forward/back, ty = height
//     // need to invert since pos will go fwd (relative to field) -> limelight up is positive
//     final double curX = -m_limelight.getEntry("ty").getDouble(0);
//     // distance to drive left/right, tx = dist left/right
//     // need to invert since pos will go left (relative to field) -> limelight left is positive
//     final double curY = -m_limelight.getEntry("tx").getDouble(0);
//     // drive: pos = ccw = left, neg = cc = right, yaw also ccw positive so don't need to do anything
//     final double curRot = m_swerveDrive.getYaw().getDegrees();
//     final double xSpeed = m_xPID.calculate(curX, 0);
//     final double ySpeed = m_yPID.calculate(curY, 0);
//     final double rotSpeed = MathUtil.clamp(m_rotPID.calculate(curRot, 180.0), -(Math.PI / 2.0), Math.PI / 2.0); // facing cones
//     // These are INVERTED (i.e. negative = up), except rotation!
//     // Make sure to take that into consideration when looking at these values.
//     SmartDashboard.putNumber("align/Current X (target right and left)", curX);
//     SmartDashboard.putNumber("align/Current Y (target down and up)", curY);
//     SmartDashboard.putNumber("align/Current Yaw (deg, +ive = left)", curRot);
//     SmartDashboard.putNumber("align/Calculated X Speed (mps [fwd])", xSpeed);
//     SmartDashboard.putNumber("align/Calculated Y Speed (mps [left]", ySpeed);
//     SmartDashboard.putNumber("align/Calculated Rot Speed (radps [left])", rotSpeed);
//     m_swerveDrive.drive(xSpeed, ySpeed, rotSpeed, true);
//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {
//     m_swerveDrive.stop();
//     setVisionTracking(false);
//   }

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     // TODO: does not actually work
//     final boolean ended = m_xPID.atSetpoint() && m_yPID.atSetpoint() && m_rotPID.atSetpoint();
//     SmartDashboard.putBoolean("align/atTarget", ended);
//     return false; // false because this doesn't actually work
//   }

//   /**
//    * Set the limelight vision tracking mode.
//    * @param isEnabled Enable or disable vision tracking.
//    */
//   static private void setVisionTracking(boolean isEnabled) {
//     // 0 = on, 1 = off
//     m_limelight.getEntry("ledMode").setNumber(isEnabled ? 0 : 1);
//     m_limelight.getEntry("camMode").setNumber(isEnabled ? 0 : 1);
//   }
// }
