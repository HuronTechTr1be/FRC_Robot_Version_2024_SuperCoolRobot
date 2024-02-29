package frc.robot.Commands;

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
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import java.util.List;

  public class DriveCommand extends Command {
  // The subsystem the command runs on
    private final DriveSubsystem m_Drive;
    private final double xSpeed;
    private final double ySpeed;
    private final double rot;

  //   private final MAXSwerveModule m_frontLeft = new MAXSwerveModule(
  //     DriveConstants.kFrontLeftDrivingCanId,
  //     DriveConstants.kFrontLeftTurningCanId,
  //     DriveConstants.kFrontLeftChassisAngularOffset);

  // private final MAXSwerveModule m_frontRight = new MAXSwerveModule(
  //     DriveConstants.kFrontRightDrivingCanId,
  //     DriveConstants.kFrontRightTurningCanId,
  //     DriveConstants.kFrontRightChassisAngularOffset);

  // private final MAXSwerveModule m_rearLeft = new MAXSwerveModule(
  //     DriveConstants.kRearLeftDrivingCanId,
  //     DriveConstants.kRearLeftTurningCanId,
  //     DriveConstants.kBackLeftChassisAngularOffset);

  // private final MAXSwerveModule m_rearRight = new MAXSwerveModule(
  //     DriveConstants.kRearRightDrivingCanId,
  //     DriveConstants.kRearRightTurningCanId,
  //     DriveConstants.kBackRightChassisAngularOffset);

  public DriveCommand(DriveSubsystem subsystem, double XSpeed, double YSpeed, double Rot) {
    m_Drive = subsystem;
    xSpeed = XSpeed;
    ySpeed = YSpeed;
    rot = Rot;
    addRequirements(m_Drive);
  }

  @Override
  public void initialize() {
    m_Drive.drive(xSpeed, ySpeed, rot, false, true);
    // while(m_frontLeft.m_drivingEncoder.getPosition()<1000){

    // }
    
}

  @Override
  public boolean isFinished() {
    return true;
  }
}
