// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

// static imports allow access to all constants in the class without using its name
import static frc.robot.Constants.Swerve.*;

import frc.robot.Constants.Swerve.Dimensions;
import frc.robot.Constants.Swerve.IsInverted;
import frc.robot.Constants.Swerve.PhysicalLimits;
import frc.robot.Constants.Swerve.TeleopLimits;
import frc.robot.utils.BeaverLogger;
import frc.robot.utils.PathLogger;
import frc.robot.Constants.CANID;
import frc.robot.Constants.Xbox;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.MathUtil;
// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
//import edu.wpi.first.util.sendable.SendableBuilder;
//import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.InstantCommand;
// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Controls all swerve drive modules.
 */
public class SwerveDrive extends SubsystemBase {
  /**
   * The yaw offset of the navx cannot be set to an arbitrary value (useful at
   * the start of autonomous), so autonomous sets the yaw offset of
   * SwerveDriveOdometry instead. Since SwerveDriveOdometry handles the yaw
   * offset, m_navx.zeroYaw() should not be called, and any navx function
   * making use of the navx's yaw should not be used (as it will be incorrect).
   * <p> The yaw should only be used for updating odometry.
   */
  private static AHRS m_navx;
  private final XboxController m_driverController;

  // x and y are relative to robot (x front/rear, y left/right)
  // private final SlewRateLimiter m_xSpeedLimiter = new SlewRateLimiter(X_SPEED_DELAY);
  // private final SlewRateLimiter m_ySpeedLimiter = new SlewRateLimiter(Y_SPEED_DELAY);
  // private final SlewRateLimiter m_rotationLimiter = new SlewRateLimiter(ROTATION_DELAY);

  private final Translation2d m_frontLeftLocation = new Translation2d(Dimensions.frontLeftX, Dimensions.frontLeftY);
  private final Translation2d m_frontRightLocation = new Translation2d(Dimensions.frontRightX, Dimensions.frontRightY);
  private final Translation2d m_rearLeftLocation = new Translation2d(Dimensions.rearLeftX, Dimensions.rearLeftY);
  private final Translation2d m_rearRightLocation = new Translation2d(Dimensions.rearRightX, Dimensions.rearRightY);
  
  private final SwerveModule m_frontLeft = new SwerveModule("Front Left", CANID.frontLeftPrimary, CANID.frontLeftSecondary, CANID.frontLeftRotation, IsInverted.frontLeftDrive, IsInverted.frontLeftRotation);
  private final SwerveModule m_frontRight = new SwerveModule("Front Right", CANID.frontRightPrimary, CANID.frontRightSecondary, CANID.frontRightRotation, IsInverted.frontRightDrive, IsInverted.frontRightRotation);
  private final SwerveModule m_rearLeft = new SwerveModule("Rear Left", CANID.rearLeftPrimary, CANID.rearLeftSecondary, CANID.rearLeftRotation, IsInverted.rearLeftDrive, IsInverted.rearLeftRotation);
  private final SwerveModule m_rearRight = new SwerveModule("Rear Right", CANID.rearRightPrimary, CANID.rearRightSecondary, CANID.rearRightRotation, IsInverted.rearRightDrive, IsInverted.rearRightRotation);
  
  private final PathLogger m_pathLogger = new PathLogger();
  private final SwerveDriveKinematics m_kinematics =
      new SwerveDriveKinematics(
          m_frontLeftLocation, m_frontRightLocation, m_rearLeftLocation, m_rearRightLocation);
  private final SwerveDriveOdometry m_odometry;
  /** Displays the robot's position relative to the field through NetworkTables. */
  private final Field2d m_field = new Field2d();

  private double m_debugAngleSetpoint = 0; // radians
  private boolean m_maxSpeedEnabled = false;
  private double m_secondaryThrottle = 0; // 0 to 1

  /** Creates a new SwerveDrive. */
  public SwerveDrive(XboxController driverController) {
    m_driverController = driverController;
    try {
      m_navx = new AHRS(SerialPort.Port.kMXP);
    } catch (RuntimeException e) {
      DriverStation.reportError("Navx initialization failed - Could not setup SwerveDriveOdometry", false);
    }
    m_odometry = new SwerveDriveOdometry(
        m_kinematics,
        m_navx.getRotation2d(),
        getModulePositions()
    );
    resetModuleEncoders();
    // group modules under this subsystem in LiveWindow
    // addChild("Front Left", m_frontLeft);
    // addChild("Front Right", m_frontRight);
    // addChild("Rear Left", m_rearLeft);
    // addChild("Rear Right", m_rearRight);
    /* The Field2d widget can also be used to visualize the robot's trajectory:
     * https://docs.wpilib.org/en/stable/docs/software/dashboards/glass/field2d-widget.html
     * contains information on how to accomplish this. This could be helpful for
     * autonomous programming. */
    SmartDashboard.putData("Field", m_field);
    SmartDashboard.putBoolean("Reset Encoders", false); // display button
    // Configure logging sources
    m_pathLogger.setSources(this::getPose);
    PPSwerveControllerCommand.setLoggingCallbacks(
        m_pathLogger::setActiveTrajectory,
        m_pathLogger::setTargetPose,
        m_pathLogger::setSetpoint,
        m_pathLogger::setError
    );
  }

  /** Configure data being sent and recieved from NetworkTables. */
  // @Override
  // public void initSendable(SendableBuilder builder) {
  //   // ()-> is a lambda that returns the value following the arrow.
  //   // It is used because addBooleanProperty() requires a Callable.
  //   builder.addBooleanProperty("Is Field Relative", ()->m_isFieldRelative, (boolean b)->{m_isFieldRelative=b;});
  //   // getter always sets value to false to reset button
  //   builder.addBooleanProperty("Reset Encoders", ()->false, (boolean pressed)->{if (pressed) resetModuleEncoders();});
  // }

  // This method will be called once per scheduler run.
  @Override
  public void periodic() {
    // odometry and NetworkTables
    updateOdometry();
    m_field.setRobotPose(m_odometry.getPoseMeters());
    m_frontLeft.updateNetworkTables();
    m_frontRight.updateNetworkTables();
    m_rearLeft.updateNetworkTables();
    m_rearRight.updateNetworkTables();
    m_rearLeft.simulateECVT();
    // handle button input from NetworkTables
    if (SmartDashboard.getBoolean("Reset Encoders", false)) {
      resetModuleEncoders();
      resetPose(new Pose2d());
      SmartDashboard.putBoolean("Reset Encoders", false); // reset the button
    }
    SmartDashboard.putNumber("Boost Multiplier", m_secondaryThrottle);
    // navx
    SmartDashboard.putBoolean("Navx Connected", m_navx.isConnected());
    SmartDashboard.putNumber("Gyro Pitch (deg)", m_navx.getPitch());
    SmartDashboard.putNumber("Gyro Roll (deg)", m_navx.getRoll());
    SmartDashboard.putNumber("Odometry Yaw (rad)", getPose().getRotation().getRadians());
    SmartDashboard.putNumber("Odometry Yaw (deg)", getPose().getRotation().getDegrees());
  }

  /** 
   * Drive the robot using given inputs.
   *
   * @param xSpeed Speed of the robot in m/s relative to the x axis of the 
   * field (or robot). (Forward/Back Speed)
   * @param ySpeed Speed of the robot in m/s relative to the y axis of the
   * field (or robot). (Left/Right Speed)
   * @param rotationSpeed How many rotations the robot does per second, in
   * radians. (Positive values are counterclockwise rotations.)
   * @param isFieldRelative Whether the robot's movement is relative to the
   * field or the robot frame.
   */
  public void drive(double xSpeed, double ySpeed, double rotationSpeed, boolean isFieldRelative) {
    // Find states using field relative position or robot relative position.
    SwerveModuleState[] states = 
        m_kinematics.toSwerveModuleStates(
            isFieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rotationSpeed, getYaw())
                : new ChassisSpeeds(xSpeed, ySpeed, rotationSpeed));
    // Prevent robot from going faster than it should.
    SwerveDriveKinematics.desaturateWheelSpeeds(states, PhysicalLimits.MAX_POSSIBLE_LINEAR_SPEED);
    setDesiredStates(states);
  }

  /**
   * Set the velocity and angle of all swerve drive modules using input from
   * the driver's Xbox controller (specified in class constructor).
   * Velocity is set to the left Y axis.
   * Angle is set to a persistent value which is increased or decreased using
   * the right X axis.
   * Use for testing purposes only.
   */
  public void debugDrive() {
    double velocity = m_driverController.getLeftY();
    m_debugAngleSetpoint += m_driverController.getRightX() * DEBUG_DRIVE_ANGLE_SENSITIVITY;
    set(velocity, m_debugAngleSetpoint);
  }

  /** 
   * Follow an autonomous trajectory. Resets odometry to path start point if 
   * this is the first path.
   * Does not trigger commands at event markers.
   * 
   * @param trajectory The trajectory to follow.
   * @param isFirstPath Whether or not this is the first path.
   * 
   * @return An InstantCommand to reset the robot pose followed by a command to
   * follow the provided trajectory.
   */
  // public Command followTrajectoryCommand(PathPlannerTrajectory trajectory, boolean isFirstPath) {
  //   // Create path-following command
  //   PPSwerveControllerCommand MP = new PPSwerveControllerCommand(
  //           trajectory,
  //           this::getPose, // Pose supplier
  //           m_kinematics, // SwerveDriveKinematics
  //           new PIDController(1, 0, 0), // X controller. Setting these values to 0 will only use feedforwards.
  //           new PIDController(1, 0, 0), // Y controller. (Usually the same values as X controller.)
  //           new PIDController(1, 0, 0), // Rotation controller. Setting these values to 0 will only use feedforwards.
  //           this::setDesiredStates, // Module states consumer
  //           true, // Should the path be automatically mirrored depending on alliance color.
  //           this // Requires this drive subsystem
  //       );
  //   // Append path-following command to an automatic odometry reset command
  //   return new SequentialCommandGroup(
  //       new InstantCommand(() -> {
  //         // Reset odometry for the first path you run during auto
  //         if(isFirstPath){
  //             resetPose(trajectory.getInitialHolonomicPose());
  //         } // not sure if this works correctly when on red team
  //       }),
  //       MP
  //   );
  // }


  /**
   * Returns a reference to the robot's gyro.
   * Do not attempt to modify or access the robot yaw using this method.
   * 
   * @return The robot's navX IMU.
   */
  public AHRS getGyro() {
    return m_navx;
  }

  /**
   * Returns the swerve drive kinematics.
   * 
   * @return The swerve drive kinematics
   */
  public SwerveDriveKinematics getKinematics() {
    return m_kinematics;
  }

  /**
   * Returns an array containing the position of each swerve module.
   * The position of a swerve module contains the drive motor position instead
   * of its velocity.
   * 
   * @return The position of each swerve module in an array.
   */
  public SwerveModulePosition[] getModulePositions() {
    return new SwerveModulePosition[] {
      m_frontLeft.getPosition(),
      m_frontRight.getPosition(),
      m_rearLeft.getPosition(),
      m_rearRight.getPosition()
    };
  }

  /**
   * Returns an array containing the state of each swerve module.
   * The state of a swerve module contains the drive motor velocity instead of
   * its position.
   * 
   * @return The state of each swerve module in an array.
   */
  public SwerveModuleState[] getModuleStates() {
    return new SwerveModuleState[] {
      m_frontLeft.getState(),
      m_frontRight.getState(),
      m_rearLeft.getState(),
      m_rearRight.getState()
    };
  }

  /**
   * Returns the current robot position in metres.
   * 
   * @return The position of the robot on the field in metres.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Returns the reported yaw of the robot according to the odometry.
   * Use this value instead of the yaw value from the navx because this will
   * take into account the autonomous starting position, meaning the angle will
   * be offset correctly during a match.
   * 
   * @return The yaw of the robot as a Rotation2d.
   */
  public Rotation2d getYaw() {
    return getPose().getRotation();
  }

  /**
   * Drive the robot using joystick inputs from the driver's Xbox controller 
   * (controller specified in class constructor).
   */
  public void manualDrive() {
    /* getLeftY() is used for xSpeed because xSpeed moves robot forward/back.
     * (The same applies for getLeftX()). This occurs because of the way robot
     * coordinates are implemented in WPILib.
     * (See https://docs.wpilib.org/en/stable/docs/software/advanced-controls/geometry/coordinate-systems.html.)
     */
    /* Speeds are inverted because Xbox controllers return negative values when
     * joystick is pushed forward or to the left.
     */
    final double xInput = MathUtil.applyDeadband(-m_driverController.getLeftY(), Xbox.JOYSTICK_DEADBAND);
    final double xSpeedSquare = xInput >= 0.0 ? xInput * xInput : -(xInput * xInput);
    final double xSpeed = xSpeedSquare * TeleopLimits.MAX_LINEAR_VELOCITY;
    // final double xSpeed =
    //     -m_xSpeedLimiter.calculate(MathUtil.applyDeadband(
    //         m_driverController.getLeftY(), Xbox.JOYSTICK_DEADBAND))
    //             * TeleopLimits.MAX_LINEAR_VELOCITY; // m/s
    //             // scale value from 0-1 to 0-MAX_LINEAR_SPEED

    final double yInput = MathUtil.applyDeadband(-m_driverController.getLeftX(), Xbox.JOYSTICK_DEADBAND);
    final double ySpeedSquare = yInput >= 0.0 ? yInput * yInput : -(yInput * yInput);
    final double ySpeed = ySpeedSquare * TeleopLimits.MAX_LINEAR_VELOCITY;
    // final double ySpeed =
    //     -m_ySpeedLimiter.calculate(MathUtil.applyDeadband(
    //         m_driverController.getLeftX(), Xbox.JOYSTICK_DEADBAND))
    //             * TeleopLimits.MAX_LINEAR_VELOCITY;

    final double rotInput = MathUtil.applyDeadband(-m_driverController.getRightX(), Xbox.JOYSTICK_DEADBAND);
    final double rotSpeedSquare = rotInput >= 0.0 ? rotInput * rotInput : -(rotInput * rotInput);
    final double rotSpeed = rotSpeedSquare * TeleopLimits.MAX_LINEAR_VELOCITY;
    // final double rotationSpeed =
    //     -m_rotationLimiter.calculate(MathUtil.applyDeadband(
    //         m_driverController.getRightX(), Xbox.JOYSTICK_DEADBAND))
    //             * TeleopLimits.MAX_ANGULAR_VELOCITY; // radians / second

    m_maxSpeedEnabled = m_driverController.getAButton();
    m_secondaryThrottle = m_driverController.getRightTriggerAxis() / 2.0;

    drive(xSpeed, ySpeed, rotSpeed, true);
  }

  /**
   * Reset the angle setpoint used for debugDrive.
   */
  public void resetDebugAngle() {
    m_debugAngleSetpoint = 0;
  }

  /**
   * Reset the encoders of each module. DOES NOT HOME THE MODULE!
   * This should be used to reset the encoder position after manually homing
   * every module.
   * 
   * This is automatically called when the robot is powered up, so it is
   * unnecessary to call this if the modules are already homed when the robot
   * is turned on.
   */
  public void resetModuleEncoders() {
    m_frontLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearLeft.resetEncoders();
    m_rearRight.resetEncoders();
  }

  /** 
   * Set the position (relative to the field) of the robot.
   * 
   * It is not necessary to reset the rotation or distance encoders, or the
   * gyro angle before calling this function (this should not be done).
   * 
   * @param pose The new position of the robot.
  */
  public void resetPose(Pose2d pose) {
    m_odometry.resetPosition(m_navx.getRotation2d(), getModulePositions(), pose);
  }

  /**
   * Manually set the velocity and angle of all swerve drive modules.
   * Use for testing purposes only.
   * 
   * @param velocity The desired velocity of the modules in metres per second.
   * @param angle The desired angle of the modules in radians.
   */
  public void set(double velocity, double angle) {
    m_frontLeft.setVelocity(velocity);
    m_frontRight.setVelocity(velocity);
    m_rearLeft.setVelocity(velocity);
    m_rearRight.setVelocity(velocity);

    m_frontLeft.rotateTo(angle);
    m_frontRight.rotateTo(angle);
    m_rearLeft.rotateTo(angle);
    m_rearRight.rotateTo(angle);
  }

  /**
   * Rotate each module to a 45 degree angle away from the centre of the robot
   * so that the robot cannot be rotated or translated. This helps to hold the
   * robot on the balance platform.
   * <p>This does not work currently.
   * 
   * @return Whether or not each module is at the setpoint.
   */
  // public boolean setBalanceLock() {
  //   return m_frontLeft.rotateTo(-Math.PI / 2.0)
  //   && m_frontRight.rotateTo(Math.PI / 2.0)
  //   && m_rearLeft.rotateTo(Math.PI / 2.0)
  //   && m_frontRight.rotateTo(-Math.PI / 2.0);
  // }

  /**
   * Set the desired state of each swerve module.
   * 
   * @param states An array containing each SwerveModuleState.
   */
  public void setDesiredStates(SwerveModuleState[] states) {
    // Array index order must match the order that m_kinematics was initialized with.
    m_frontLeft.setDesiredState(states[0], m_secondaryThrottle, m_maxSpeedEnabled);
    m_frontRight.setDesiredState(states[1], m_secondaryThrottle, m_maxSpeedEnabled);
    m_rearLeft.setDesiredState(states[2], m_secondaryThrottle, m_maxSpeedEnabled);
    m_rearRight.setDesiredState(states[3], m_secondaryThrottle, m_maxSpeedEnabled);
    // BeaverLogger.getInstance().logMP(m_pathLogger, states, getModuleStates());
  }

  public void setDesiredStatesAuto(SwerveModuleState[] states) {
    // Array index order must match the order that m_kinematics was initialized with.
    m_frontLeft.setDesiredStateAuto(states[0], m_secondaryThrottle, m_maxSpeedEnabled);
    m_frontRight.setDesiredStateAuto(states[1], m_secondaryThrottle, m_maxSpeedEnabled);
    m_rearLeft.setDesiredStateAuto(states[2], m_secondaryThrottle, m_maxSpeedEnabled);
    m_rearRight.setDesiredStateAuto(states[3], m_secondaryThrottle, m_maxSpeedEnabled);
    BeaverLogger.getInstance().logMP(m_pathLogger, states, getModuleStates());
  }

  /**
   * Rotate each module to the specified angle.
   * 
   * @param angle The module angle to move to in radians.
   * @return Whether or not all modules have finished rotating.
   */
  public boolean setRotationAngle(double angle){
    return m_frontLeft.rotateTo(angle) && m_frontRight.rotateTo(angle) 
        && m_rearLeft.rotateTo(angle) && m_rearRight.rotateTo(angle);
  }

  public void setRotationBreakMode(boolean isBreak){
    m_frontLeft.setRotationBreakMode(isBreak);
    m_frontRight.setRotationBreakMode(isBreak);
    m_rearLeft.setRotationBreakMode(isBreak);
    m_rearRight.setRotationBreakMode(isBreak);
  } // Map to user button?

  /**
   * Stop all swerve modules.
   */
  public void stop() {
    m_frontLeft.stop();
    m_frontRight.stop();
    m_rearLeft.stop();
    m_rearRight.stop();
  }

  /** Updates the field relative position of the robot. */
  public void updateOdometry() {
    m_odometry.update(m_navx.getRotation2d(), getModulePositions());
  }

  /**
   * Reset the yaw reported by SwerveDriveOdometry.
   */
  public void zeroYaw() {
    resetPose(new Pose2d(m_odometry.getPoseMeters().getTranslation(), new Rotation2d(0)));
  }
}
