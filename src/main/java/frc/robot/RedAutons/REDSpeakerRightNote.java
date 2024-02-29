package frc.robot.RedAutons;
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
import frc.robot.Commands.DriveCommand;
import frc.robot.Commands.DriveCommandDistance;
import frc.robot.Commands.DriveTimed;
import frc.robot.Commands.HighShootCommand;
import frc.robot.Commands.LowShootCommand;
import frc.robot.Commands.ResetWheelPositionCommand;
import frc.robot.Commands.MotorsStillCommand;
import frc.robot.Commands.PickUpCommand;
import frc.robot.Commands.RejectCommand;
import frc.robot.Commands.StopDriveCommand;
import frc.robot.Commands.TrajectoryForwardAuton;
import frc.robot.Commands.TrajectoryBackwardAuton;
import frc.robot.Commands.HighShootCommand;
import frc.robot.Commands.HighShootTimed;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.RobotContainer;
import frc.robot.Subsystems.DriveSubsystem;
import frc.robot.Subsystems.EgressSubsystem;
import frc.robot.Subsystems.FlapSubsystem;
import frc.robot.Subsystems.IntakeModule;
import frc.robot.Subsystems.SweeperWheelsSubsystem;
import frc.robot.Subsystems.ClawSubsystem;  
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import java.util.List;
import frc.robot.Commands.HighShootCommand;

public class REDSpeakerRightNote extends SequentialCommandGroup     {
    
  private RobotContainer m_robotContainer;

    public REDSpeakerRightNote(DriveSubsystem drive, EgressSubsystem shoot, IntakeModule conveyorBelt, SweeperWheelsSubsystem sweepers, FlapSubsystem flap){
        addCommands(
          
          
          new DriveTimed(drive, -0.2, 0, 0, 0.2),

          new WaitCommand(0.1),

          new HighShootTimed(shoot, conveyorBelt, sweepers,flap, 0.6),
            new WaitCommand(0.2),

           new DriveTimed(drive, 0, 0, -0.3, 0.30),

           new WaitCommand(0.1),

            new PickUpCommand(shoot, conveyorBelt, sweepers),
            new WaitCommand(.1), 
            new ResetWheelPositionCommand(drive),        
            new DriveCommandDistance(drive, -0.5, 0, 0, 1.70), 
            new WaitCommand(0.3),
            new MotorsStillCommand(shoot, conveyorBelt, sweepers,flap),
            new WaitCommand(0.1),
            new ResetWheelPositionCommand(drive),        
            new DriveCommandDistance(drive, 0.5, 0, 0, 1.70),         
            new WaitCommand(.2), 
            new DriveTimed(drive, 0, 0, 0.3, 0.35),
            new WaitCommand(.1),
            //new DriveTimed(drive, .2, 0, 0, .20),
            new WaitCommand(0.1),
            new HighShootTimed(shoot, conveyorBelt, sweepers,flap, 0.6),
            new WaitCommand(0.1),
            //new DriveTimed(drive, 0, 0, -0.3, 0.1),
            new WaitCommand(0.1),
            new ResetWheelPositionCommand(drive),
            new DriveCommandDistance(drive, -0.5, 0, 0, 2.2)
             
            );
    }

    

}