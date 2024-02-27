// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Autons.AmpRightNote;
import frc.robot.Autons.AmpRightNote;
import frc.robot.Autons.SpeakerLeftNote;
import frc.robot.Autons.SpeakerLeftShootRetreat;
import frc.robot.Autons.SpeakerMiddleNote;
import frc.robot.Commands.FlapDownCommand;
import frc.robot.Commands.FlapUpCommand;
import frc.robot.Commands.HighShootCommand;
import frc.robot.Commands.LowShootCommand;
import frc.robot.Commands.MotorsStillCommand;
import frc.robot.Commands.PickUpCommand;
import frc.robot.Commands.RejectCommand;
//import frc.robot.Commands.HighShootCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Subsystems.DriveSubsystem;
import frc.robot.Subsystems.EgressSubsystem;
import frc.robot.Subsystems.FlapSubsystem;
import frc.robot.Subsystems.IntakeModule;
import frc.robot.Subsystems.MAXSwerveModule;
import frc.robot.Subsystems.SweeperWheelsSubsystem;
import frc.robot.Subsystems.ClawSubsystem;  
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import java.util.List;
//import frc.robot.Commands.HighShootCommand;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private EgressSubsystem m_Shoot = new EgressSubsystem();
  private IntakeModule m_conveyorBelt = new IntakeModule(33);
  private SweeperWheelsSubsystem m_SweeperWheels = new SweeperWheelsSubsystem();
  private final FlapSubsystem m_robotFlap = new FlapSubsystem(51);


  // The driver's controller
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  PS4Controller m_shooterController = new PS4Controller(1);

  JoystickButton CrossButton = new JoystickButton(m_shooterController, PS4Controller.Button.kCross.value);
  JoystickButton CircleButton = new JoystickButton(m_shooterController, PS4Controller.Button.kCircle.value);
  JoystickButton SquareButton = new JoystickButton(m_shooterController, PS4Controller.Button.kSquare.value);
  JoystickButton TriangleButton = new JoystickButton(m_shooterController, PS4Controller.Button.kTriangle.value);
  JoystickButton ShooterLeftBumper = new JoystickButton(m_shooterController, PS4Controller.Button.kL2.value);
  JoystickButton ShooterRightBumper = new JoystickButton(m_shooterController, PS4Controller.Button.kR2.value);
  JoystickButton ShooterLeftTrigger = new JoystickButton(m_shooterController, PS4Controller.Button.kL1.value);
  JoystickButton ShooterRightTrigger = new JoystickButton(m_shooterController, PS4Controller.Button.kR1.value);



  HighShootCommand HighShoot = new HighShootCommand(m_Shoot, m_conveyorBelt);
  LowShootCommand LowShoot = new LowShootCommand(m_Shoot, m_conveyorBelt);
  RejectCommand Reject = new RejectCommand(m_Shoot, m_conveyorBelt, m_SweeperWheels);
  PickUpCommand PickUp = new PickUpCommand(m_Shoot, m_conveyorBelt, m_SweeperWheels);
  FlapUpCommand FlapUp = new FlapUpCommand(m_robotFlap);
  FlapDownCommand FlapDown = new FlapDownCommand(m_robotFlap);
  MotorsStillCommand MotorsStill = new MotorsStillCommand(m_Shoot, m_conveyorBelt, m_SweeperWheels, m_robotFlap);
  SpeakerMiddleNote MiddleSpeaker = new SpeakerMiddleNote(m_robotDrive, m_Shoot, m_conveyorBelt, m_SweeperWheels,m_robotFlap);
  AmpRightNote ampNote = new AmpRightNote(m_robotDrive, m_Shoot, m_conveyorBelt, m_SweeperWheels, m_robotFlap);
  AmpRightNote ampNoteTesting = new AmpRightNote(m_robotDrive, m_Shoot, m_conveyorBelt, m_SweeperWheels, m_robotFlap);
  SpeakerLeftShootRetreat speakerLeftShootRetreat = new SpeakerLeftShootRetreat(m_robotDrive, m_Shoot, m_conveyorBelt, m_SweeperWheels, m_robotFlap);
  SpeakerLeftNote speakerLeftNote = new SpeakerLeftNote(m_robotDrive, m_Shoot, m_conveyorBelt, m_SweeperWheels, m_robotFlap);

  public void periodic(){
    if(!(CrossButton.getAsBoolean()||SquareButton.getAsBoolean()||CircleButton.getAsBoolean()||TriangleButton.getAsBoolean()||ShooterLeftBumper.getAsBoolean()||ShooterRightBumper.getAsBoolean()||ShooterLeftTrigger.getAsBoolean()||ShooterRightTrigger.getAsBoolean())){
      m_Shoot.Still();
      m_SweeperWheels.Still();
      m_conveyorBelt.Still();
      m_robotFlap.FlapStill();
    }
    SmartDashboard.putBoolean("Flap Limit", m_robotFlap.isRaised());
    
  }

  public void FlapRun(){
    
    if(ShooterRightBumper.getAsBoolean()){
      m_robotFlap.FlapUp(0.5);
    }
    else if(ShooterRightTrigger.getAsBoolean()){
      m_robotFlap.FlapDown();
    }
    else{
      m_robotFlap.FlapStill();
    }

  }

  public void initReset(){
    m_robotDrive.resetFrontRightEncoder();
  }

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {


    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
                false, true),
            m_robotDrive));
  }

  public void resetRobot(){
    
    m_robotDrive.resetClaws();
    m_robotFlap.flapSetZero();

  }

  


  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */ 
  private void configureButtonBindings() {
    new JoystickButton(m_driverController, XboxController.Button.kX.value)
        .whileTrue(new RunCommand(
            () -> m_robotDrive.setX(),
            m_robotDrive));
        CircleButton.whileTrue(HighShoot);
        SquareButton.whileTrue(LowShoot);
        CrossButton.whileTrue(Reject);
        TriangleButton.whileTrue(PickUp);
        // ShooterRightTrigger.whileTrue(FlapUp);
        // ShooterRightBumper.whileTrue(FlapDown);

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Create config for trajectory
    TrajectoryConfig config = new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(DriveConstants.kDriveKinematics);

    // An example trajectory to follow. All units in meters.
    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(new Translation2d(0,1 )),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(0, 2, new Rotation2d(0)),
        config);

    var thetaController = new ProfiledPIDController(
        AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
        exampleTrajectory,
        m_robotDrive::getPose, // Functional interface to feed supplier
        DriveConstants.kDriveKinematics,

        // Position controllers
        new PIDController(AutoConstants.kPXController, 0, 0),
        new PIDController(AutoConstants.kPYController, 0, 0),
        thetaController,
        m_robotDrive::setModuleStates,
        m_robotDrive);

    // Reset odometry to the starting pose of the trajectory.
    m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, false, false));
  }

  public Command getMiddleSpeakerAuton(){
    //return MiddleSpeaker;
    //return ampNote;
    return ampNoteTesting;
    //return speakerLeftShootRetreat;
    //return speakerLeftNote;
  }

}