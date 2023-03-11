// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

// static imports allow access to all constants in the class without using its name
import static frc.robot.Constants.Swerve.*;

import java.util.List;

import frc.robot.Constants.Swerve.CanID;
import frc.robot.Constants.Swerve.Dimensions;
import frc.robot.Constants.Swerve.PhysicalLimits;
import frc.robot.Constants.Swerve.TeleopLimits;
import frc.robot.utils.BeaverLogger;
import frc.robot.utils.PathLogger;
import frc.robot.Constants.Autonomous;
import frc.robot.Constants.Xbox;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
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
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.InstantCommand;
// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Controls all swerve drive modules.
 */
public class SwerveDrive extends SubsystemBase {
  private final AHRS m_gyro;
  private final XboxController m_driverController;

  // x and y are relative to robot (x front/rear, y left/right)
  private final SlewRateLimiter m_xSpeedLimiter = new SlewRateLimiter(X_SPEED_DELAY);
  private final SlewRateLimiter m_ySpeedLimiter = new SlewRateLimiter(Y_SPEED_DELAY);
  private final SlewRateLimiter m_rotationLimiter = new SlewRateLimiter(ROTATION_DELAY);

  private final Translation2d m_frontLeftLocation = new Translation2d(Dimensions.frontLeftX, Dimensions.frontLeftY);
  private final Translation2d m_frontRightLocation = new Translation2d(Dimensions.frontRightX, Dimensions.frontRightY);
  private final Translation2d m_rearLeftLocation = new Translation2d(Dimensions.rearLeftX, Dimensions.rearLeftY);
  private final Translation2d m_rearRightLocation = new Translation2d(Dimensions.rearRightX, Dimensions.rearRightY);
  
  private final SwerveModule m_frontLeft = new SwerveModule("Front Left", CanID.frontLeftDrive, CanID.frontLeftRotation);
  private final SwerveModule m_frontRight = new SwerveModule("Front Right", CanID.frontRightDrive, CanID.frontRightRotation);
  private final SwerveModule m_rearLeft = new SwerveModule("Rear Left", CanID.rearLeftDrive, CanID.rearLeftRotation);
  private final SwerveModule m_rearRight = new SwerveModule("Rear Right", CanID.rearRightDrive, CanID.rearRightRotation);
  
  private final SwerveAutoBuilder m_autoBuilder;
  private final PathLogger m_pathLogger = new PathLogger();

  private final SwerveDriveKinematics m_kinematics =
      new SwerveDriveKinematics(
          m_frontLeftLocation, m_frontRightLocation, m_rearLeftLocation, m_rearRightLocation);
  
  private final SwerveDriveOdometry m_odometry;
  /** Displays the robot's position relative to the field through NetworkTables. */
  private final Field2d m_field = new Field2d();

  private boolean m_isFieldRelative = false;
  private double m_debugAngleSetpoint = 0; // radians

  /** Creates a new SwerveDrive. */
  public SwerveDrive(AHRS gyro, XboxController driverController) {
    m_driverController = driverController;
    m_gyro = gyro;
    if (!m_gyro.isConnected()) {
      DriverStation.reportError(
          "Navx not initialized - Could not setup SwerveDriveOdometry", false);
    }
    m_odometry = new SwerveDriveOdometry(
        m_kinematics,
        m_gyro.getRotation2d(),
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
    // Setup autonomous command
    m_autoBuilder = new SwerveAutoBuilder(
      this::getPose,
      this::resetPose,
      m_kinematics,
      Autonomous.translationPIDConstants,
      Autonomous.rotationPIDConstants,
      this::setDesiredStates,
      Autonomous.eventMap,
      true,
      this
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
    // handle button input from NetworkTables
    m_isFieldRelative = SmartDashboard.getBoolean("Is Field Relative", false);
    SmartDashboard.putBoolean("Is Field Relative", m_isFieldRelative);
    if (SmartDashboard.getBoolean("Reset Encoders", false)) {
      resetModuleEncoders();
      SmartDashboard.putBoolean("Reset Encoders", false); // reset the button
    }
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
                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rotationSpeed, m_gyro.getRotation2d())
                : new ChassisSpeeds(xSpeed, ySpeed, rotationSpeed));
    // Prevent robot from going faster than it should.
    SwerveDriveKinematics.desaturateWheelSpeeds(states, PhysicalLimits.MAX_POSSIBLE_LINEAR_SPEED);
    setDesiredStates(states);
  }

  /** Generates a trajectory following command which drives from the current
   * position to the destination pose.
   * 
   * @param destination The Pose2d to drive to.
   * 
   * @return Trajectory following command which drives from the current
   * position to the destination pose.
   */
  public Command generateDriveToPose(Pose2d destination) {
    // should match teleop limits, problem is we don't have a teleop accel limit
    final double maxVelocity = 3.8;
    final double maxAccel = 100; // idk find the teleop desired
    final Pose2d currentPose = getPose();
    // may generate weird paths, there is no "next control" component for code-generated paths
    // create a new path and try playing around with the dot on a stick connected to waypoints, you'll see what I mean
    // ^ it allows you to create much smoother paths, unfortunately we can't generate those here
    PathPlannerTrajectory trajectory = PathPlanner.generatePath(
        new PathConstraints(maxVelocity, maxAccel),
        // position, heading, holonomic rotation
        new PathPoint(currentPose.getTranslation(), new Rotation2d(0), currentPose.getRotation()),
        new PathPoint(destination.getTranslation(), new Rotation2d(0), destination.getRotation())
    );
    // put these constants in Constants.java if this works
    return new PPSwerveControllerCommand(
            trajectory,
            this::getPose, // Pose supplier
            m_kinematics, // SwerveDriveKinematics
            new PIDController(1, 0, 0),
            new PIDController(1, 0, 0),
            new PIDController(1, 0, 0),
            this::setDesiredStates, // Module states consumer
            false, // do not mirror the path depending on alliance colour
            this // Requires this drive subsystem
        );
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
   * Follow a trajectory with markers.
   * Markers are defined by the path.
   * Marker-command bindings are specified in `Constants.Autonomous.eventMap`.
   * 
   * @param trajectory The trajectory to follow.
   * 
   * @return A command which follows the trajectory while triggering commands
   * at path-defined markers.
   */
  public Command generateFullAuto(PathPlannerTrajectory trajectory) {
    return m_autoBuilder.fullAuto(trajectory);
  }

  /**
   * Follow a group of trajectories with markers.
   * Markers are defined by each individual path.
   * Marker-command bindings are specified in `Constants.Autonomous.eventMap`.
   * 
   * @param trajectoryGroup A list of trajectories to follow.
   * 
   * @return A command which follows each trajectory while triggering commands
   * at path-defined markers.
   */
  public Command generateFullAuto(List<PathPlannerTrajectory> trajectoryGroup) {
    return m_autoBuilder.fullAuto(trajectoryGroup);
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
    final double xSpeed =
        -m_xSpeedLimiter.calculate(MathUtil.applyDeadband(
            m_driverController.getLeftY(), Xbox.JOYSTICK_DEADBAND))
                * TeleopLimits.MAX_LINEAR_VELOCITY; // m/s
                // scale value from 0-1 to 0-MAX_LINEAR_SPEED

    final double ySpeed =
        -m_ySpeedLimiter.calculate(MathUtil.applyDeadband(
            m_driverController.getLeftX(), Xbox.JOYSTICK_DEADBAND))
                * TeleopLimits.MAX_LINEAR_VELOCITY;

    final double rotationSpeed =
        -m_rotationLimiter.calculate(MathUtil.applyDeadband(
            m_driverController.getRightX(), Xbox.JOYSTICK_DEADBAND))
                * TeleopLimits.MAX_ANGULAR_VELOCITY; // radians / second

    drive(xSpeed, ySpeed, rotationSpeed, m_isFieldRelative);
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
   * Reset the position (relative to the field) of the robot.
   * 
   * It is not necessary to reset the rotation or distance encoders, or the
   * gyro angle before calling this function (this should not be done).
   * 
   * @param pose The new position of the robot.
  */
  public void resetPose(Pose2d pose) {
    m_odometry.resetPosition(m_gyro.getRotation2d(), getModulePositions(), pose);
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
   * Set the desired state of each swerve module.
   * 
   * @param states An array containing each SwerveModuleState.
   */
  public void setDesiredStates(SwerveModuleState[] states) {
    // Array index order must match the order that m_kinematics was initialized with.
    m_frontLeft.setDesiredState(states[0]);
    m_frontRight.setDesiredState(states[1]);
    m_rearLeft.setDesiredState(states[2]);
    m_rearRight.setDesiredState(states[3]);
    BeaverLogger.getInstance().logMP(m_pathLogger, states, getModuleStates());
  }

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
    m_odometry.update(m_gyro.getRotation2d(), getModulePositions());
  }
}
