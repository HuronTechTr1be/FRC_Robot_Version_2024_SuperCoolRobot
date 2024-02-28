package frc.robot.BlueAutons;
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
import frc.robot.Commands.FlapDownCommand;

import frc.robot.Commands.FlapUpCommand;
import frc.robot.Commands.HighShootCommand;
import frc.robot.Commands.LowShootCommand;
import frc.robot.Commands.MotorsStillCommand;
import frc.robot.Commands.PickUpCommand;
import frc.robot.Commands.RejectCommand;
import frc.robot.Commands.ResetWheelPositionCommand;
import frc.robot.Commands.StopDriveCommand;
import frc.robot.Commands.TrajectoryForwardAuton;
import frc.robot.Commands.TrajectoryBackwardAuton;
import frc.robot.Commands.HighShootCommand;
import frc.robot.Commands.HighShootTimed;
import frc.robot.Commands.LowShootTimed;
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

// MAKE SURE YOU ARE RIGHT AGAINST THE AMP

public class BLUEAmpRightNote extends SequentialCommandGroup     {
    
  private RobotContainer m_robotContainer;

    public BLUEAmpRightNote(DriveSubsystem drive, EgressSubsystem shoot, IntakeModule conveyorBelt, SweeperWheelsSubsystem sweepers, FlapSubsystem flap){
        addCommands(
            
            //Whole other Method
            new FlapUpCommand(flap),
            new WaitCommand(0.1),
            new LowShootTimed(shoot, conveyorBelt, sweepers, flap,0.9, 0.3),
            new FlapDownCommand(flap),

            ////option 1
            new ResetWheelPositionCommand(drive), 
            new DriveCommandDistance(drive, -0.5, 0, 0, 0.6),
            
            new WaitCommand(0.3),

            new DriveTimed(drive, 0, 0, 0.4, 0.5),

            new WaitCommand(0.3), 
            new PickUpCommand(shoot, conveyorBelt, sweepers),
            
            ////option 1
            new ResetWheelPositionCommand(drive), 
            new DriveCommandDistance(drive, -0.5, 0, 0, -0.8),

            
            new WaitCommand(0.2),
            new MotorsStillCommand(shoot, conveyorBelt, sweepers,flap),
            new WaitCommand(0.3),

            ////option 1
            new ResetWheelPositionCommand(drive), 
            new DriveCommandDistance(drive, 0.5, 0, 0, 1),
            
            
            new WaitCommand(0.3),


            ////option 2
            new DriveTimed(drive, 0, 0, -0.4, 0.45),
            
            new WaitCommand(0.3),

            ////option 1
            new ResetWheelPositionCommand(drive), 
            new DriveCommandDistance(drive, 0.5, 0, 0, 0.8),
            ////option 2
            //new DriveTimed(drive, 0.5, 0, 0, 1),

            //new WaitCommand(.3),
           // new DriveTimed(drive, 0, 0, 0.2, 0.3),

            new FlapUpCommand(flap),
            new WaitCommand(0.1),
            new LowShootTimed(shoot, conveyorBelt, sweepers, flap, 0.6, 0.35),
            new FlapDownCommand(flap),

            new ResetWheelPositionCommand(drive), 
            new DriveCommandDistance(drive, -0.5, 0, 0, 0.7),


            new WaitCommand(0.3),

            new DriveTimed(drive, 0, 0, 0.4, 0.6),

            new WaitCommand(0.1),

            new ResetWheelPositionCommand(drive), 
            new DriveCommandDistance(drive, -0.5, 0, 0, 1)
            

            );
    }

    

}
