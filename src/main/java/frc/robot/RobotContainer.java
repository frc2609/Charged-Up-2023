// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;
import java.util.Map;
import static java.util.Map.entry;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
// import edu.wpi.first.wpilibj2.command.button.Trigger;
// import frc.robot.commands.AdjustUpperSetpoint;
import frc.robot.commands.CloseGripper;
import frc.robot.commands.ExampleAuto;
import frc.robot.commands.ManualArmControl;
import frc.robot.commands.ManualDrive;
import frc.robot.commands.MoveArmToGroundPickup;
import frc.robot.commands.MoveArmToMid;
import frc.robot.commands.MoveArmToMidProfiled;
import frc.robot.commands.MoveArmToHigh;
import frc.robot.commands.MoveArmToLow;
import frc.robot.commands.MoveArmToPickup;
import frc.robot.commands.MoveArmToStow;
import frc.robot.commands.OpenGripper;
import frc.robot.commands.ResetModules;
import frc.robot.commands.VisionAlign;
import frc.robot.commands.autonomous.AutoScoreConeHigh;
import frc.robot.commands.autonomous.AutoScoreConeThenBalance;
import frc.robot.subsystems.ArmGripper;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.utils.BeaverLogger;
import frc.robot.utils.PathLogger;

import com.pathplanner.lib.commands.PPSwerveControllerCommand;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  /** Entries in this map must be non-null, or the program will crash. */
  
  final HashMap<String, Command> m_eventMap = new HashMap<>();

  private static AHRS m_navx;
  private final XboxController m_driverController = new XboxController(
      Constants.Xbox.DRIVER_CONTROLLER_PORT);
  private final XboxController m_operatorController = new XboxController(
      Constants.Xbox.OPERATOR_CONTROLLER_PORT);
  /* Subsystems should be marked as private so they can only be accessed by
   * commands that require them. This prevents a subsystem from being used by
   * multiple things at once, which may potentially cause issues. */
  private final ArmGripper m_armGripper;
  private final SwerveDrive m_swerveDrive;
  private final PowerDistribution m_powerDistribution =
      new PowerDistribution(1, ModuleType.kRev);
  
  // driver controls
  // private final JoystickButton m_fieldOrientedToggleButton = 
  //     new JoystickButton(
  //         m_driverController, XboxController.Button.kBack.value);
  // private final JoystickButton m_resetEncoderButton =
  //     new JoystickButton(
  //         m_driverController, XboxController.Button.kStart.value);
  private final JoystickButton m_zeroYawButton =
      new JoystickButton(m_driverController, XboxController.Button.kY.value);
  private final JoystickButton m_driverPickup = new JoystickButton(
      m_driverController, XboxController.Button.kRightBumper.value);
  private final JoystickButton m_driverGroundPickup = new JoystickButton(
      m_driverController, XboxController.Button.kLeftBumper.value);
  private final JoystickButton m_driverStow = new JoystickButton(
      m_driverController, XboxController.Button.kX.value);
  // operator controls
  private final JoystickButton m_openGripper = new JoystickButton(
      m_operatorController, XboxController.Button.kLeftBumper.value);
  private final JoystickButton m_closeGripper = new JoystickButton(
      m_operatorController, XboxController.Button.kRightBumper.value);
  // private final JoystickButton m_pickupButton = new JoystickButton(
  //     m_operatorController, XboxController.Button.kLeftStick.value);
  // private final JoystickButton m_groundPickupButton = new JoystickButton(
  //     m_operatorController, XboxController.Button.kRightStick.value);
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

      private final JoystickButton visionButton = new JoystickButton(m_driverController, XboxController.Button.kB.value);
      private final JoystickButton resetButton = new JoystickButton(m_driverController, XboxController.Button.kA.value);
      private final JoystickButton breakButton = new JoystickButton(m_driverController, XboxController.Button.kStart.value);
  PPSwerveControllerCommand tempAutoCommand;
  PathLogger m_pathLogger;
  public static Spark LED;
  // private final Trigger m_decreaseUpperSetpoint = new Trigger(() -> { return m_operatorController.getLeftTriggerAxis() >= 0.5; });
  // private final Trigger m_increaseUpperSetpoint = new Trigger(() -> { return m_operatorController.getLeftTriggerAxis() >= 0.5; });
          
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    try {
      m_navx = new AHRS(SerialPort.Port.kMXP);
    } catch (RuntimeException e) {
      DriverStation.reportError("Navx initialization failed", false);
    }
    m_armGripper = new ArmGripper(m_operatorController);
    m_pathLogger = new PathLogger();
    m_eventMap.put("CloseGripper", new CloseGripper(m_armGripper)); // markerName, Command
    m_eventMap.put("MoveArmToStow", new MoveArmToStow(m_armGripper));
    m_swerveDrive = new SwerveDrive(m_navx, m_driverController, m_eventMap);
    configureButtonBindings();
    SmartDashboard.putBoolean("Zero Yaw", false); // display the button
    m_swerveDrive.resetModuleEncoders();
    
    LED = new Spark(1);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // driver controls
    // replaced with SmartDashboard buttons
    // m_fieldOrientedToggleButton.onTrue(new InstantCommand(
    //   () -> m_isFieldRelative = !m_isFieldRelative));
    // m_resetEncoderButton.onTrue(new InstantCommand(
    //   m_swerveDrive::resetModuleEncoders, m_swerveDrive));
    // this one left in for easy access to resetYaw
    m_zeroYawButton.onTrue(new InstantCommand(m_navx::zeroYaw));
    m_driverGroundPickup.onTrue(new MoveArmToGroundPickup(m_armGripper));
    m_driverPickup.onTrue(new MoveArmToPickup(m_armGripper));
    m_driverStow.onTrue(new MoveArmToStow(m_armGripper));
    // operator controls
    m_stowButton.onTrue(new MoveArmToStow(m_armGripper));
    // m_pickupButton.onTrue(new MoveArmToPickup(m_armGripper));
    // m_groundPickupButton.onTrue(new MoveArmToGroundPickup(m_armGripper));
    m_scoreLowButton.onTrue(new MoveArmToLow(m_armGripper));
    m_scoreMidButton.onTrue(new MoveArmToMid(m_armGripper));
    m_scoreHighButton.onTrue(new MoveArmToHigh(m_armGripper));
    m_closeGripper.onTrue(new CloseGripper(m_armGripper));
    m_openGripper.onTrue(new OpenGripper(m_armGripper));
    // m_decreaseUpperSetpoint.onTrue(new AdjustUpperSetpoint(m_armGripper, -2.0));
    // m_increaseUpperSetpoint.onTrue(new AdjustUpperSetpoint(m_armGripper, 2.0));
    // this will interrupt any running arm commands, is this a good idea?
    // also, the operator will lose control of the arm when open or close gripper is scheduled.
    // TODO: move Gripper into own subsystem so that these don't cancel arm commands
    m_toggleManualControl.toggleOnTrue(new ManualArmControl(m_armGripper));
    visionButton.whileTrue(new VisionAlign(m_swerveDrive, m_driverController));
    breakButton.whileTrue(new MoveArmToMidProfiled(m_armGripper));
    // breakButton.onTrue(new AutoScoreConeHigh(m_armGripper));
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
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return new AutoScoreConeThenBalance(m_armGripper, m_swerveDrive);
    // return null;
  }

  //TODO: Temp til Antoine puts on absolute encoders
  public void setRotToCoast(){
    m_swerveDrive.setRotCoast();
  }

  /**
   * Update NetworkTables values set by RobotContainer.
   */
  public void updateNetworkTables() {
    SmartDashboard.putNumber("Gyro Angle", m_navx.getAngle());
    SmartDashboard.putBoolean("Navx Connected", m_navx.isConnected());
    if (SmartDashboard.getBoolean("Zero Yaw", false)) {
      m_navx.zeroYaw();
      SmartDashboard.putBoolean("Zero Yaw", false); // reset the button
    }
    SmartDashboard.putNumber("Robot Current Draw (A)", m_powerDistribution.getTotalCurrent());
  }

  public void resetEncoders(){
    m_swerveDrive.resetModuleEncoders();
  }
}