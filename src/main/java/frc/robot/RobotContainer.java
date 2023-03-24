// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.Autonomous;
import frc.robot.Constants.Swerve.AutonomousLimits;
import frc.robot.commands.Autobalance;
import frc.robot.commands.ManualArmControl;
import frc.robot.commands.ManualDrive;
import frc.robot.commands.MoveArmToGroundPickup;
import frc.robot.commands.MoveArmToMid;
import frc.robot.commands.MoveArmToHigh;
import frc.robot.commands.MoveArmToLow;
import frc.robot.commands.MoveArmToPickup;
import frc.robot.commands.MoveArmToStow;
import frc.robot.commands.QueueCommand;
import frc.robot.commands.ResetModules;
import frc.robot.commands.VisionAlign;
import frc.robot.commands.autonomous.ScoreConeHigh;
import frc.robot.subsystems.ArmGripper;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.SwerveDrive;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  /** Entries in this map must be non-null, or the program will crash. */
  private final HashMap<String, Command> m_eventMap = new HashMap<>();
  private final SendableChooser<PathPlannerTrajectory> m_pathChooser = new SendableChooser<>();
  /* Subsystems should be marked as private so they can only be accessed by
   * commands that require them. This prevents a subsystem from being used by
   * multiple things at once, which may potentially cause issues. */
  private final ArmGripper m_armGripper;
  private final SwerveDrive m_swerveDrive;
  private final SwerveAutoBuilder m_autoBuilder;
  private final PowerDistribution m_powerDistribution =
      new PowerDistribution(1, ModuleType.kRev);
  private final XboxController m_driverController = new XboxController(
      Constants.Xbox.DRIVER_CONTROLLER_PORT);
  private final XboxController m_operatorController = new XboxController(
      Constants.Xbox.OPERATOR_CONTROLLER_PORT);
  
  // driver controls
  private final JoystickButton m_zeroYawButton = new JoystickButton(
      m_driverController, XboxController.Button.kStart.value);
  private final JoystickButton m_driverPickup = new JoystickButton(
      m_driverController, XboxController.Button.kRightBumper.value);
  private final JoystickButton m_driverGroundPickup = new JoystickButton(
      m_driverController, XboxController.Button.kLeftBumper.value);
  private final JoystickButton m_driverStow = new JoystickButton(
      m_driverController, XboxController.Button.kX.value);
  private final JoystickButton m_executeQueuedCommand = new JoystickButton(
      m_driverController, XboxController.Button.kY.value);
  private final JoystickButton m_alignToNode = new JoystickButton(
      m_driverController, XboxController.Button.kB.value);

  // operator controls
  private final JoystickButton m_openGripper = new JoystickButton(
      m_operatorController, XboxController.Button.kLeftBumper.value);
  private final JoystickButton m_closeGripper = new JoystickButton(
      m_operatorController, XboxController.Button.kRightBumper.value);
  private final JoystickButton m_stowButton = new JoystickButton(
      m_operatorController, XboxController.Button.kX.value);
  private final JoystickButton m_scoreLowButton = new JoystickButton(
      m_operatorController, XboxController.Button.kA.value);
  private final JoystickButton m_scoreMidButton = new JoystickButton(
      m_operatorController, XboxController.Button.kB.value);
  private final JoystickButton m_scoreHighButton = new JoystickButton(
      m_operatorController, XboxController.Button.kY.value);
  private final JoystickButton m_toggleManualControl = new JoystickButton(
      m_operatorController, XboxController.Button.kStart.value);
  private final JoystickButton m_resetSwerveModules = new JoystickButton(
      m_operatorController, XboxController.Button.kBack.value);
  private final JoystickButton m_requestCone = new JoystickButton(
      m_operatorController, XboxController.Button.kLeftStick.value);
  private final JoystickButton m_requestCube = new JoystickButton(
      m_operatorController, XboxController.Button.kRightStick.value);
          
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    m_armGripper = new ArmGripper(m_operatorController);
    m_swerveDrive = new SwerveDrive(m_driverController);
    m_swerveDrive.resetModuleEncoders();
    configureButtonBindings();
    configureEventMap();
    configurePathChooser();
    m_autoBuilder = new SwerveAutoBuilder(
      m_swerveDrive::getPose,
      m_swerveDrive::resetPose,
      m_swerveDrive.getKinematics(),
      Autonomous.translationPIDConstants,
      Autonomous.rotationPIDConstants,
      m_swerveDrive::setDesiredStatesAuto,
      m_eventMap,
      true,
      m_swerveDrive
    );
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // driver controls
    m_zeroYawButton.onTrue(new InstantCommand(m_swerveDrive::zeroYaw));
    m_driverGroundPickup.onTrue(new MoveArmToGroundPickup(m_armGripper));
    m_driverPickup.onTrue(new MoveArmToPickup(m_armGripper));
    m_driverStow.onTrue(new MoveArmToStow(m_armGripper));
    m_alignToNode.whileTrue(new VisionAlign(m_swerveDrive, m_driverController));
    // operator controls
    m_stowButton.onTrue(new MoveArmToStow(m_armGripper));
    m_scoreLowButton.onTrue(new QueueCommand(m_executeQueuedCommand, new MoveArmToLow(m_armGripper)));
    m_scoreMidButton.onTrue(new QueueCommand(m_executeQueuedCommand, new MoveArmToMid(m_armGripper)));
    m_scoreHighButton.onTrue(new QueueCommand(m_executeQueuedCommand, new MoveArmToHigh(m_armGripper)));
    m_closeGripper.onTrue(new InstantCommand(m_armGripper::closeGripper));
    m_openGripper.onTrue(new InstantCommand(m_armGripper::openGripper));
    m_resetSwerveModules.onTrue(new ResetModules(m_swerveDrive, 0));
    m_requestCone.onTrue(new InstantCommand(LED::setCone));
    m_requestCube.onTrue(new InstantCommand(LED::setCube));
    // TODO: move Gripper into own subsystem so that these don't cancel arm commands
    m_toggleManualControl.toggleOnTrue(new ManualArmControl(m_armGripper));
  }

  /** 
   * Add markers to the autonomous event map.
   */
  private void configureEventMap() {
    m_eventMap.put("Autobalance", new Autobalance(m_swerveDrive));
    m_eventMap.put("MoveArmToStow", new MoveArmToStow(m_armGripper));
    m_eventMap.put("ScoreHigh", new ScoreConeHigh(m_swerveDrive, m_armGripper));
  }

  /**
   * Load possible autonomous paths.
   */
  private void configurePathChooser() {
    PathConstraints constraints = new PathConstraints(AutonomousLimits.MAX_LINEAR_VELOCITY, AutonomousLimits.MAX_LINEAR_ACCELERATION);
    /* 
     * Do not include filepath or extension in path name.
     * File path assumed to be `src/main/deploy/pathplanner/`.
     * Extension assumed to be `.path`.
     */
    m_pathChooser.setDefaultOption("ScoreThenAutobalance", PathPlanner.loadPath("ScoreThenAutobalance", constraints));
    m_pathChooser.addOption("ScoreThenDriveOut", PathPlanner.loadPath("ScoreThenDriveOut", constraints));
    m_pathChooser.addOption("ScoreThenDriveOutAndRotate", PathPlanner.loadPath("ScoreThenDriveOutAndRotate", constraints));
    SmartDashboard.putData(m_pathChooser);
  }

  /**
   * Disable driver control of the drivetrain.
   * <p>Should be called at the start of autonomous to prevent driver control
   * during autonomous after the robot is switched from teleop to autonomous
   * mode. If this is not called, whenever autonomous is not using the
   * drivetrain, the driver will have control of the robot during autonomous.
   * This situation won't be encountered during a match, but may cause issues
   * during testing or development.
   */
  public void disableTeleopControl() {
    // m_armGripper.setDefaultCommand(null);
    m_swerveDrive.setDefaultCommand(null);
  }

  /**
   * Set the default command of the drivetrain to driver control.
   * <p>Should be called at the start of teleop to allow the driver to control
   * the robot.
   */
  public void enableTeleopControl() {
    /* Using a default command instead of calling the manualDrive() function in
     * teleopPeriodic() allows a command to take over the drivetrain
     * temporarily during teleop. This may be useful for auto-balancing or
     * moving into position to deliver a game piece. */
    // m_armGripper.setDefaultCommand(new ManualArmControl(m_armGripper));
    m_swerveDrive.setDefaultCommand(new ManualDrive(m_swerveDrive));
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(1);
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(1);
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(9);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return The selected autonomous command.
   */
  public Command getAutonomousCommand() {
    return m_autoBuilder.fullAuto(m_pathChooser.getSelected());
  }

  //TODO: Temp til Antoine puts on absolute encoders
  public void setRotToCoast() {
    m_swerveDrive.setRotCoast();
  }

  /**
   * Update NetworkTables values set by RobotContainer.
   */
  public void updateNetworkTables() {
    SmartDashboard.putNumber("Robot Current Draw (A)", m_powerDistribution.getTotalCurrent());
  }
}