// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.server.PathPlannerServer;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.MP.Looper;
import frc.robot.subsystems.LED;
import frc.robot.utils.AbsoluteEncoderHandler;
import frc.robot.utils.TunableNumber;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private RobotContainer robotContainer;
	private Looper enabledLooper;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    /* These methods log NetworkTables values & DS data to a WPILOG
     * file that can be converted to a CSV or viewed with AdvantageScope. */
    DataLogManager.start(); // log NetworkTables values
    DriverStation.startDataLog(DataLogManager.getLog()); // log DS data

    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    robotContainer = new RobotContainer();
    robotContainer.setArmBrake(true);
    PathPlannerServer.startServer(Constants.Autonomous.PATHPLANNER_SERVER_PORT);

    enabledLooper = new Looper();
    try {
      enabledLooper.register(robotContainer.getArmLoop());
    } catch(Throwable t) {
      System.out.println(t.getMessage());
      System.out.println(t.getStackTrace());
    }
    // robotContainer.armLEDSetup(false);
    SmartDashboard.putBoolean("Arm Brake", true);
    SmartDashboard.putBoolean("Rotation Brake", true);
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
    LED.getInstance().periodic();
    AbsoluteEncoderHandler.updateAllVelocities();
    robotContainer.updateNetworkTables();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    robotContainer.setArmBrake(true);
    robotContainer.setDriveBrake(true);
		enabledLooper.stop();
  }

  @Override
  public void disabledPeriodic() {
    robotContainer.setArmBrake(SmartDashboard.getBoolean("Arm Brake", true));
    robotContainer.setDriveBrake(SmartDashboard.getBoolean("Rotation Brake", true));
    TunableNumber.updateAll();
  }

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    // Prevent the robot from being driven by the default command during auto.
    robotContainer.disableTeleopControl();
    m_autonomousCommand = robotContainer.getAutonomousCommand();
    robotContainer.setArmBrake(true);
    robotContainer.setDriveBrake(true);

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
    
		enabledLooper.start();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    robotContainer.enableTeleopControl();
    robotContainer.setArmBrake(true);
    robotContainer.setDriveBrake(true);
    
		enabledLooper.start();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
    robotContainer.disableTeleopControl();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
