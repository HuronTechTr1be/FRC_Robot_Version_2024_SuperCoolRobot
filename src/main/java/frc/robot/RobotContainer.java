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
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Subsystems.DriveSubsystem;
import frc.robot.Subsystems.FlapSubsystem;
import frc.robot.Subsystems.ClawSubsystem;  
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import java.util.List;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  //private final FlapSubsystem m_robotFlap = new FlapSubsystem(51);


  // The driver's controller
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);

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
    //m_robotFlap.flapSetZero();

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
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  // public Command getAutonomousCommand() {
  //   // Create config for trajectory
  //   TrajectoryConfig config = new TrajectoryConfig(
  //       AutoConstants.kMaxSpeedMetersPerSecond,
  //       AutoConstants.kMaxAccelerationMetersPerSecondSquared)
  //       // Add kinematics to ensure max speed is actually obeyed
  //       .setKinematics(DriveConstants.kDriveKinematics);

  //   // An example trajectory to follow. All units in meters.
  //   Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
  //       // Start at the origin facing the +X direction
  //       new Pose2d(0, 0, new Rotation2d(0)),
  //       // Pass through these two interior waypoints, making an 's' curve path
  //       List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
  //       // End 3 meters straight ahead of where we started, facing forward
  //       new Pose2d(3, 0, new Rotation2d(0)),
  //       config);

  //   var thetaController = new ProfiledPIDController(
  //       AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
  //   thetaController.enableContinuousInput(-Math.PI, Math.PI);

  //   SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
  //       exampleTrajectory,
  //       m_robotDrive::getPose, // Functional interface to feed supplier
  //       DriveConstants.kDriveKinematics,

  //       // Position controllers
  //       new PIDController(AutoConstants.kPXController, 0, 0),
  //       new PIDController(AutoConstants.kPYController, 0, 0),
  //       thetaController,
  //       m_robotDrive::setModuleStates,
  //       m_robotDrive);

  //   // Reset odometry to the starting pose of the trajectory.
  //   m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

  //   // Run path following command, then stop at the end.
  //   return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, false, false));
  // }

  public Command getAutonomousCommand() {
    // ---- This section is used to create drive constants that will be referenced later --- JNL
    // Create config for trajectory
    TrajectoryConfig config = new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(DriveConstants.kDriveKinematics);

        // ---- This section of code should be use to make the list of steps to move ---- JNL
        ///  ----  this could be the first example for if we are on the left side. ---- JNL
    // An example trajectory to follow. All units in meters.
    Trajectory speakerMiddleNoteSEQ = TrajectoryGenerator.generateTrajectory(
        new Pose2d(0, 0, new Rotation2d(0)),
          // high shoot human given note
        List.of(
            new Translation2d(-1, 0), // pick up while backing up to retreve the middle note
            new Translation2d(0,0) // stop picking up wile moving back to 0,0
          ),
          new Pose2d(0, 0, new Rotation2d(0)), 
            // high shoot retreved note
          config);

        // ---- second sequence that can be used if on the right side ... or whatever  ---  JNL
        Trajectory speakerRightNoteSEQ = TrajectoryGenerator.generateTrajectory(

            // basically the same as speakerMiddleNote but we will add rotation to pick up the right note
        new Pose2d(0, 0, new Rotation2d(0)),
          // high shoot human given note
        List.of(
            new Translation2d(-1, 0), // pick up while backing up to retreve the right note
            new Translation2d(0,0) // stop picking up wile moving back to 0,0
          ),
          new Pose2d(0, 0, new Rotation2d(0)), 
            // high shoot retreved note
          config);

          Trajectory speakerLeftNoteSEQ = TrajectoryGenerator.generateTrajectory(

            // basically the same as speakerMiddleNote but we will add rotation to pick up the left note
        new Pose2d(0, 0, new Rotation2d(0)),
          // high shoot human given note
        List.of(
            new Translation2d(-1, 0), // pick up while backing up to retreve the left note
            new Translation2d(0,0) // stop picking up wile moving back to 0,0
          ),
          new Pose2d(0, 0, new Rotation2d(0)), 
            // high shoot retreved note
          config);

        //----   Need to create a new step type that gives you pickup and another typ of shoot.  like Pose2d...   ---- JNL

         Trajectory exampleTrajectory; 
          //   --- do some magic to have a switch to select what version you run. ------    JNL

          double autonOption = SmartDashboard.getNumber("Auton Option", 0);

            if (autonOption == 1) 
              exampleTrajectory = speakerMiddleNoteSEQ;

            if (autonOption == 2) 
              exampleTrajectory = speakerRightNoteSEQ;

            if (autonOption == 3) 
              exampleTrajectory = speakerLeftNoteSEQ;

            else
              exampleTrajectory = speakerMiddleNoteSEQ;

          //exampleTrajectory = speakerMiddleNote;

        //---- create X & Y motor PIDs to tune the control of the closed loop control for these parameters --- JNL
    var thetaController = new ProfiledPIDController(
        AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);


    // at this point you need to choose one of the SEQ options above and stuff it into the conatainer here - - JNL

          // ---- Briing all of the things above together into a container ---- JNL
    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
      // at this point you need to choose one of the SEQ options above and stuff it into the conatainer here - - JNL

      exampleTrajectory,
        m_robotDrive::getPose, // Functional interface to feed supplier
        DriveConstants.kDriveKinematics,

        // Position controllers
        new PIDController(AutoConstants.kPXController, 0, 0),
        new PIDController(AutoConstants.kPYController, 0, 0),
        thetaController,
        m_robotDrive::setModuleStates,
        m_robotDrive);

      // ---- As stated below - clears the old distance and rotation values from previous call ----  JNL  
    // Reset odometry to the starting pose of the trajectory.
    m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, false, false));
  }

}