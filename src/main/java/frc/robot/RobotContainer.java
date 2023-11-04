// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

// import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.MP.Loop;
import frc.robot.Constants.Autonomous;
import frc.robot.Constants.Xbox;
import frc.robot.Constants.Swerve.AutonomousLimits;
import frc.robot.commands.arm.ManualArmControl;
import frc.robot.commands.arm_positions.MoveArmToLow;
// import frc.robot.commands.arm_positions.MoveArmToLow;
import frc.robot.commands.arm_positions.MoveArmToStow;
import frc.robot.commands.arm_positions.ShortThrowMid;
import frc.robot.commands.arm_positions.StowMidToHigh;
import frc.robot.commands.auto.ScoreConeHigh;
import frc.robot.commands.gripper.AutoGroundPickup;
import frc.robot.commands.gripper.AutoShelfPickup;
import frc.robot.commands.swerve.Autobalance;
import frc.robot.commands.utility.QueueCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.SwerveDrive;
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
  private final HashMap<String, Command> m_eventMap = new HashMap<>();
  private final SendableChooser<PathPlannerTrajectory> pathChooser = new SendableChooser<>();

  /*
   * Subsystems should be marked as private so they can only be accessed by
   * commands that require them. This prevents a subsystem from being used by
   * multiple things at once, which may potentially cause issues.
   */
  private final Arm arm;
  private final Gripper gripper;
  private final SwerveDrive drive;
  private final SwerveAutoBuilder autoBuilder;

  private final CommandXboxController driverController = new CommandXboxController(Xbox.driverControllerPort);
  private final CommandXboxController operatorController = new CommandXboxController(Xbox.operatorControllerPort);
  private final Trigger executeQueuedCommand = driverController.y();
  private final Trigger reverseArmPath = driverController.y();

  private final PowerDistribution powerDistribution;
  private final BeaverLogger logger = new BeaverLogger();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // CameraServer.startAutomaticCapture();
    arm = new Arm();
    gripper = new Gripper();
    drive = new SwerveDrive(null);
    drive.resetModuleEncoders();

    if (RobotBase.isReal()) {
      powerDistribution = new PowerDistribution(1, PowerDistribution.ModuleType.kRev);
    } else {
      powerDistribution = null;
    }

    configureButtonBindings();
    configureDefaultCommands();
    configureEventMap();
    configurePathChooser();
    configureLoggedData();

    autoBuilder = new SwerveAutoBuilder(
        drive::getPose,
        drive::resetPose,
        drive.getKinematics(),
        Autonomous.translationPIDConstants,
        Autonomous.rotationPIDConstants,
        drive::setDesiredStatesAuto,
        m_eventMap,
        true,
        drive);
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
    // driver
    driverController.start().onTrue(new InstantCommand(drive::zeroYaw));
    driverController.leftBumper().onTrue(new AutoGroundPickup(arm, gripper));
    driverController.rightBumper().onTrue(new AutoShelfPickup(arm, gripper, operatorController));
    driverController.x().onTrue(new MoveArmToStow(arm));
    // operator
    operatorController.start().onTrue(new InstantCommand(() -> {}, arm));
    operatorController.leftBumper().onTrue(new InstantCommand(gripper::openGripper));
    operatorController.rightBumper().onTrue(new InstantCommand(gripper::closeGripper));
    operatorController.a().onTrue(new MoveArmToLow(arm));
    operatorController.b().onTrue(new QueueCommand(executeQueuedCommand, new ShortThrowMid(arm, reverseArmPath, operatorController)));
    operatorController.y().onTrue(new QueueCommand(executeQueuedCommand, new StowMidToHigh(arm, reverseArmPath, operatorController)));
    operatorController.x().onTrue(new MoveArmToStow(arm));
    // LEDs
    // blink LEDs while held
    operatorController.leftStick().whileTrue(new InstantCommand(LED.getInstance()::setUrgentCone));
    // set solid while not held (when button no longer held sets to solid)
    operatorController.leftStick().onFalse(new InstantCommand(LED.getInstance()::setCone));
    operatorController.rightStick().whileTrue(new InstantCommand(LED.getInstance()::setUrgentCube));
    operatorController.rightStick().onFalse(new InstantCommand(LED.getInstance()::setCube));
  }

  private void configureDefaultCommands() {
    arm.setDefaultCommand(new ManualArmControl(arm, operatorController));
    // arm.setDefaultCommand(new ManualDashboardArmControl(arm));
  }

  /**
   * Add markers to the autonomous event map.
   */
  private void configureEventMap() {
    m_eventMap.put("Autobalance", new Autobalance(drive));
    m_eventMap.put("MoveArmToStow", new MoveArmToStow(arm));
    m_eventMap.put("ScoreHigh", new ScoreConeHigh(drive, arm, gripper));
    m_eventMap.put("CubePickup", new AutoGroundPickup(arm, gripper));
  }

  /**
   * Load possible autonomous paths.
   */
  private void configurePathChooser() {
    PathConstraints constraints = new PathConstraints(AutonomousLimits.MAX_LINEAR_VELOCITY,
        AutonomousLimits.MAX_LINEAR_ACCELERATION);
    /*
     * Do not include filepath or extension in path name.
     * File path assumed to be `src/main/deploy/pathplanner/`.
     * Extension assumed to be `.path`.
     */
    pathChooser.setDefaultOption("ScoreThenAutobalance", PathPlanner.loadPath("ScoreThenAutobalance", constraints));
    pathChooser.addOption("ScoreThenDriveOut", PathPlanner.loadPath("ScoreThenDriveOut", constraints));
    pathChooser.addOption("ScoreThenDriveOutAndRotate",
        PathPlanner.loadPath("ScoreThenDriveOutAndRotate", constraints));
    pathChooser.addOption("ConeCubeAuto", PathPlanner.loadPath("ConeCubeAuto", constraints));
    pathChooser.addOption("ClearSide2piece", PathPlanner.loadPath("ClearSide2piece", constraints));
    SmartDashboard.putData(pathChooser);
  }

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
  public void disableTeleopControl() {
    arm.removeDefaultCommand();
    drive.removeDefaultCommand();
    gripper.removeDefaultCommand();
  }

  /**
   * Set the default command of the drivetrain to driver control.
   * <p>
   * Should be called at the start of teleop to allow the driver to control
   * the robot.
   */
  public void enableTeleopControl() {
    configureDefaultCommands();
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return The selected autonomous command.
   */
  public Command getAutonomousCommand() {
    return autoBuilder.fullAuto(pathChooser.getSelected()).andThen(new InstantCommand(drive::stop));
  }

  // public void armLEDSetup(boolean initial) {
  //     m_armGripper.setupLED(initial);
  // }

  public Loop getArmLoop() {
    return arm.getLoop();
  }

  public void setArmBrake(boolean isBrake) {
    arm.setBrake(isBrake);
  }

  public void setDriveBrake(boolean isBrake) {
    drive.setRotationBrake(isBrake);
  }

  /**
   * Update NetworkTables values set by RobotContainer.
   */
  public void updateNetworkTables() {
    // arm.logger.logAll();
    logger.logAll();
  }
}