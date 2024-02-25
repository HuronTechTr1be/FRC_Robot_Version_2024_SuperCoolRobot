// HI!
// IDK refers to temperary code that we are not possitive works or matters OR we are working on making comments there


// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
 
import edu.wpi.first.wpilibj.TimedRobot;
//import edu.wpi.first.wpilibj.Timer;
//import edu.wpi.first.wpilibj.XboxController;
//import edu.wpi.first.wpilibj.drive.DifferentialDrive;
//import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
/*import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import frc.robot.Constants.OIConstants;*/
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoSink;
import edu.wpi.first.cscore.VideoSource.ConnectionStrategy;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj.XboxController;
//import main.java.frc.robot.RobotContainer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.BeltConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.EgressConstants;
import frc.robot.Constants.SweeperWheelConstants;
import frc.robot.Subsystems.ClawSubsystem;
import frc.robot.Subsystems.EgressSubsystem;
import frc.robot.Subsystems.IntakeModule;
import frc.robot.Subsystems.SweeperWheelsSubsystem;
import frc.robot.Subsystems.FlapSubsystem;
import frc.robot.AutonSwitch;
import frc.robot.Autons.SpeakerMiddleNote;
import edu.wpi.first.cameraserver.CameraServer;

 /**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends TimedRobot {

  private Command m_autonomousCommand;
  

  private RobotContainer m_robotContainer;
  private AutonSwitch m_autonSwitch;
  //private ClawSubsystem m_clawLeft = new ClawSubsystem(21);
  //private ClawSubsystem m_clawRight = new ClawSubsystem(22);
  
  //private FlapSubsystem m_flap = new FlapSubsystem(51);

  

  DigitalInput autonSwitchInput = new DigitalInput(0);

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {

    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    
     //CameraServer.startAutomaticCapture();

    CameraServer.startAutomaticCapture(0);

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

   }
 
  /** This function is  called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}
/** a */
  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  /** This function is run once each time the robot enters autonomous mode. */
  @Override
  public void autonomousInit() {

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

  /** This function is called once each time the robot enters teleoperated mode. */
  @Override
  public void teleopInit() {

    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    m_robotContainer.resetRobot();
    SmartDashboard.setDefaultString("FlapDown", "Default");
  }

  /** This function is called periodically during operator control. */
  /** This function is called periodically during teleoperated mode. */
  @Override
  public void teleopPeriodic() {

     m_robotContainer.periodic();
     m_robotContainer.FlapRun();


  }

  /** This function is called once each time the robot enters test mode. */
  @Override
  public void testInit() {

    SmartDashboard.setDefaultNumber("Speed Factor", DriveConstants.kSpeedFactor);
    SmartDashboard.setDefaultNumber("Rotate Factor", DriveConstants.kRotateFactor);
    


    // SmartDashboard.putNumber("Bottom Low Shoot Factor:", EgressConstants.id41LowShootFactor);
    // SmartDashboard.putNumber("Bottom High Shoot Factor:", EgressConstants.id41HighShootFactor);
    // SmartDashboard.putNumber("Top Low Shoot Factor:", EgressConstants.id42LowShootFactor);
    // SmartDashboard.putNumber("Top High Shoot Factor:", EgressConstants.id42HighShootFactor);
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();

  }

  /** This function is called periodically during test mode.  */
  @Override
  public void testPeriodic() {
    
    m_robotContainer.periodic();  
    //m_robotContainer.FlapRun();  

    double bottomHighShootFactor = SmartDashboard.getNumber("Bottom High Shoot Factor:", EgressConstants.id41HighShootFactor);
    SmartDashboard.setDefaultNumber("Bottom High Shoot Factor:", bottomHighShootFactor);
    if(Math.abs(bottomHighShootFactor)>1){
      bottomHighShootFactor = 0;
    }
    double topHighShootFactor = SmartDashboard.getNumber("Top High Shoot Factor:", EgressConstants.id42HighShootFactor);
    SmartDashboard.setDefaultNumber("Top High Shoot Factor:", topHighShootFactor);
    if(Math.abs(topHighShootFactor)>1){
      topHighShootFactor = 0;
    }
    double bottomLowShootFactor = SmartDashboard.getNumber("Bottom Low Shoot Factor:", EgressConstants.id41LowShootFactor);
    SmartDashboard.setDefaultNumber("Bottom Low Shoot Factor:", bottomLowShootFactor);
    if(Math.abs(bottomLowShootFactor)>1){
      bottomLowShootFactor = 0;
    }
    double topLowShootFactor = SmartDashboard.getNumber("Top Low Shoot Factor:", EgressConstants.id42LowShootFactor);
    SmartDashboard.setDefaultNumber("Top Low Shoot Factor:", topLowShootFactor);
    if(Math.abs(topLowShootFactor)>1){
      topLowShootFactor = 0;
    }

    // if (drive2Controller.getCircleButton()) {
    //   if(bottomHighShootFactor==EgressConstants.id41HighShootFactor && topHighShootFactor==EgressConstants.id42HighShootFactor){
    //   m_topShoot.HighShoot();
    //   m_bottomShoot.HighShoot();
    //   m_conveyorBelt.HighShoot();
    //   }
    //   else{
    //   m_topShoot.adjustedHighShoot(topHighShootFactor);
    //   m_bottomShoot.adjustedHighShoot(bottomHighShootFactor);
    //   m_conveyorBelt.HighShoot();  
    //   }
      
    // }
    // else if (drive2Controller.getSquareButton()) {
    //   if(bottomLowShootFactor==EgressConstants.id41LowShootFactor && topLowShootFactor==EgressConstants.id42LowShootFactor){
    //   m_topShoot.LowShoot();
    //   m_bottomShoot.LowShoot();
    //   m_conveyorBelt.LowShoot();
    //   }
    //   else{
    //   m_topShoot.adjustedLowShoot(topLowShootFactor);
    //   m_bottomShoot.adjustedLowShoot(bottomLowShootFactor);
    //   m_conveyorBelt.LowShoot();
    //   }
    // }
    // else if (drive2Controller.getCrossButton()) {
    //   m_topShoot.Reject();
    //   m_bottomShoot.Reject();
    //   m_conveyorBelt.reject();
    //   m_leftSweeperWheel.Reject(); 
    //   m_rightSweeperWheel.Reject();
    // } 
    //  else if (drive2Controller.getTriangleButton()) {;
    //   m_conveyorBelt.PickUp();
    //   m_leftSweeperWheel.PickUp();
    //   m_rightSweeperWheel.PickUp();
    //   m_bottomShoot.PickUp();
    // }
    // else {
    //   m_topShoot.Still();
    //   m_bottomShoot.Still();
    //   m_conveyorBelt.still();
    //   m_leftSweeperWheel.Still();
    //   m_rightSweeperWheel.Still();
    // }

  /*


 */
  
  }
}
    

  



