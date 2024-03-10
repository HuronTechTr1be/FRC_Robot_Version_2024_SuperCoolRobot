// HI!
// IDK refers to temperary code that we are not possitive works or matters OR we are working on making comments there

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoSink;
import edu.wpi.first.cscore.VideoSource.ConnectionStrategy;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.EgressConstants;
import frc.robot.Subsystems.LEDSubsystem;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the manifest
 * file in the resource
 * directory.
 */
public class Robot extends TimedRobot {

  private Command m_autonomousCommand;

  private LEDSubsystem m_LedSubsystem;
  private RobotContainer m_robotContainer;

  UsbCamera camera1;
  //UsbCamera camera2;
  VideoSink server;

  DigitalInput autonSwitchInput = new DigitalInput(0);


  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  @Override
  public void robotInit() {

    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();

    SmartDashboard.putNumber("Auton Picker", 0);
    m_LedSubsystem = new LEDSubsystem();

    

  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items
   * like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and
   * SmartDashboard integrated updating.
   */

  @Override
  public void robotPeriodic() {
    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled
    // commands, running already-scheduled commands, removing finished or
    // interrupted commands,
    // and running subsystem periodic() methods. This must be called from the
    // robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();

  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
  }

  /** a */
  @Override
  public void disabledPeriodic() {

    // if (ally.get() == Alliance.Blue) {
    //   m_LedSubsystem.setAll(Color.kBlue);
    // } else if (ally.get() == Alliance.Red) {
    //   m_LedSubsystem.setAll(Color.kRed);
    // } 
      m_LedSubsystem.rainbow();
    

  }

  /**
   * This autonomous runs the autonomous command selected by your
   * {@link RobotContainer} class.
   */
  /** This function is run once each time the robot enters autonomous mode. */
  @Override
  public void autonomousInit() {

    m_robotContainer.resetReverseDrive();
    m_autonomousCommand = m_robotContainer.getMiddleSpeakerAuton();

    /*
     * String autoSelected = SmartDashboard.getString("Auto Selector",
     * "Default"); switch(autoSelected) { case "My Auto": autonomousCommand
     * = new MyAutoCommand(); break; case "Default Auto": default:
     * autonomousCommand = new ExampleCommand(); break; }
     */

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }

  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {

  }

  /**
   * This function is called once each time the robot enters teleoperated mode.
   */
  @Override
  public void teleopInit() {

    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.

    camera1 = CameraServer.startAutomaticCapture(0);
    //camera2 = CameraServer.startAutomaticCapture(1);
    server = CameraServer.getServer();

    camera1.setConnectionStrategy(ConnectionStrategy.kKeepOpen);
    //camera2.setConnectionStrategy(ConnectionStrategy.kKeepOpen);

    Optional<Alliance> ally = DriverStation.getAlliance();

    if (ally.get() == Alliance.Blue) {
      m_LedSubsystem.setAll(Color.kBlue);
    } else if (ally.get() == Alliance.Red) {
      m_LedSubsystem.setAll(Color.kRed);
    } 


    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  /** This function is called periodically during teleoperated mode. */
  @Override
  public void teleopPeriodic() {

    m_robotContainer.periodic();
    m_robotContainer.FlapRun();
    //m_robotContainer.cameraSwitch(camera1, camera2, server);

  }

  /** This function is called once each time the robot enters test mode. */
  @Override
  public void testInit() {

    SmartDashboard.setDefaultNumber("Speed Factor", DriveConstants.kSpeedFactor);
    SmartDashboard.setDefaultNumber("Rotate Factor", DriveConstants.kRotateFactor);
    m_robotContainer.resetRobot();

    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();

  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {

    m_robotContainer.periodic();

    double bottomHighShootFactor = SmartDashboard.getNumber("Bottom High Shoot Factor:",
        EgressConstants.id41HighShootFactor);
    SmartDashboard.setDefaultNumber("Bottom High Shoot Factor:", bottomHighShootFactor);
    if (Math.abs(bottomHighShootFactor) > 1) {
      bottomHighShootFactor = 0;
    }
    double topHighShootFactor = SmartDashboard.getNumber("Top High Shoot Factor:", EgressConstants.id42HighShootFactor);
    SmartDashboard.setDefaultNumber("Top High Shoot Factor:", topHighShootFactor);
    if (Math.abs(topHighShootFactor) > 1) {
      topHighShootFactor = 0;
    }
    double bottomLowShootFactor = SmartDashboard.getNumber("Bottom Low Shoot Factor:",
        EgressConstants.id41LowShootFactor);
    SmartDashboard.setDefaultNumber("Bottom Low Shoot Factor:", bottomLowShootFactor);
    if (Math.abs(bottomLowShootFactor) > 1) {
      bottomLowShootFactor = 0;
    }
    double topLowShootFactor = SmartDashboard.getNumber("Top Low Shoot Factor:", EgressConstants.id42LowShootFactor);
    SmartDashboard.setDefaultNumber("Top Low Shoot Factor:", topLowShootFactor);
    if (Math.abs(topLowShootFactor) > 1) {
      topLowShootFactor = 0;
    }

    

  }
}
