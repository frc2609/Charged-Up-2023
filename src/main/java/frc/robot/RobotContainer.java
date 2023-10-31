// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.MP.Loop;
import frc.robot.Constants.Xbox;
import frc.robot.commands.arm.ArmPositions;
// import frc.robot.commands.arm.ManualArmControl;
import frc.robot.commands.arm.ManualDashboardArmControl;
import frc.robot.commands.arm.MoveArmToPosition;
import frc.robot.subsystems.Arm;
import frc.robot.utils.BeaverLogger;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  /** Entries in this map must be non-null, or the program will crash. */
  // private final HashMap<String, Command> m_eventMap = new HashMap<>();
  // private final SendableChooser<PathPlannerTrajectory> m_pathChooser = new SendableChooser<>();

  /*
   * Subsystems should be marked as private so they can only be accessed by
   * commands that require them. This prevents a subsystem from being used by
   * multiple things at once, which may potentially cause issues.
   */
  private final Arm arm;

  private final CommandXboxController driverController = new CommandXboxController(Xbox.driverControllerPort);
  private final CommandXboxController operatorController = new CommandXboxController(Xbox.operatorControllerPort);

  private final PowerDistribution powerDistribution;

  // private final SwerveAutoBuilder m_autoBuilder;

  private final BeaverLogger logger = new BeaverLogger();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    CameraServer.startAutomaticCapture();
    arm = new Arm();

    if (RobotBase.isReal()) {
      powerDistribution = new PowerDistribution(1, PowerDistribution.ModuleType.kRev);
    } else {
      powerDistribution = null;
    }

    configureButtonBindings();
    configureDefaultCommands();
    configureEventMap();
    // configurePathChooser();
    configureLoggedData();

    // m_autoBuilder = new SwerveAutoBuilder(
    //     m_swerveDrive::getPose,
    //     m_swerveDrive::resetPose,
    //     m_swerveDrive.getKinematics(),
    //     Autonomous.translationPIDConstants,
    //     Autonomous.rotationPIDConstants,
    //     m_swerveDrive::setDesiredStatesAuto,
    //     m_eventMap,
    //     true,
    //     m_swerveDrive);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
   * it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    operatorController.x().onTrue(new MoveArmToPosition(arm, ArmPositions.testPosition, false));
  }

  private void configureDefaultCommands() {
    // arm.setDefaultCommand(new ManualArmControl(
    //     arm,
    //     operatorController::getLeftY,
    //     () -> -operatorController.getRightY(),
    //     operatorController::getLeftTriggerAxis,
    //     operatorController::getRightTriggerAxis
    // ));
    arm.setDefaultCommand(new ManualDashboardArmControl(arm));
  }

  /**
   * Add markers to the autonomous event map.
   */
  private void configureEventMap() {
    // m_eventMap.put("Autobalance", new Autobalance(m_swerveDrive));
    // m_eventMap.put("MoveArmToStow", new MoveArmToStow(m_armGripper));
    // m_eventMap.put("ScoreHigh", new ScoreConeHigh(m_swerveDrive, m_armGripper));
    // m_eventMap.put("CubePickup", new GroundPickCube(m_armGripper));
  }

  // /**
  //  * Load possible autonomous paths.
  //  */
  // private void configurePathChooser() {
  //   PathConstraints constraints = new PathConstraints(AutonomousLimits.MAX_LINEAR_VELOCITY,
  //       AutonomousLimits.MAX_LINEAR_ACCELERATION);
  //   /*
  //    * Do not include filepath or extension in path name.
  //    * File path assumed to be `src/main/deploy/pathplanner/`.
  //    * Extension assumed to be `.path`.
  //    */
  //   m_pathChooser.setDefaultOption("ScoreThenAutobalance", PathPlanner.loadPath("ScoreThenAutobalance", constraints));
  //   m_pathChooser.addOption("ScoreThenDriveOut", PathPlanner.loadPath("ScoreThenDriveOut", constraints));
  //   m_pathChooser.addOption("ScoreThenDriveOutAndRotate",
  //       PathPlanner.loadPath("ScoreThenDriveOutAndRotate", constraints));
  //   m_pathChooser.addOption("ConeCubeAuto", PathPlanner.loadPath("ConeCubeAuto", constraints));
  //   m_pathChooser.addOption("ClearSide2piece", PathPlanner.loadPath("ClearSide2piece", constraints));
  //   SmartDashboard.putData(m_pathChooser);
  // }

  private void configureLoggedData() {
    if (powerDistribution != null) {
      logger.addLoggable("PDP Voltage", powerDistribution::getVoltage, false);
      logger.addLoggable("PDP Total Current", powerDistribution::getTotalCurrent, false);
    }
  }

  /**
   * Disable driver control of the drivetrain.
   * <p>
   * Should be called at the start of autonomous to prevent driver control
   * during autonomous after the robot is switched from teleop to autonomous
   * mode. If this is not called, whenever autonomous is not using the
   * drivetrain, the driver will have control of the robot during autonomous.
   * This situation won't be encountered during a match, but may cause issues
   * during testing or development.
   */
  public void disableTeleopControl() {}

  /**
   * Set the default command of the drivetrain to driver control.
   * <p>
   * Should be called at the start of teleop to allow the driver to control
   * the robot.
   */
  public void enableTeleopControl() {
    /*
     * Using a default command instead of calling the manualDrive() function in
     * teleopPeriodic() allows a command to take over the drivetrain
     * temporarily during teleop. This may be useful for auto-balancing or
     * moving into position to deliver a game piece.
     */
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return The selected autonomous command.
   */
  public Command getAutonomousCommand() {
    return null;
    // return m_autoBuilder.fullAuto(m_pathChooser.getSelected()).andThen(new InstantCommand(m_swerveDrive::stop));
    // PathConstraints constraints = new
    // PathConstraints(AutonomousLimits.MAX_LINEAR_VELOCITY, 1);

    // return m_autoBuilder.fullAuto(PathPlanner.loadPath("DriveStraight",
    // constraints));
  }

  public void armLEDSetup(boolean initial) {
    // m_armGripper.setupLED(initial);
  }

  public Loop getArmLoop() {
    return arm.getLoop();
  }

  public void setArmBrake(boolean isBrake) {
    arm.setBrake(isBrake);
  }

  /**
   * Update NetworkTables values set by RobotContainer.
   */
  public void updateNetworkTables() {
    arm.logger.logAll();
    logger.logAll();
  }
}