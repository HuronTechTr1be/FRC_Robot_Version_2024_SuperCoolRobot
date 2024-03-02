package frc.robot.BlueAutons;
import frc.robot.Commands.DriveCommandDistance;
import frc.robot.Commands.DriveTimed;
import frc.robot.Commands.ResetWheelPositionCommand;
import frc.robot.Commands.MotorsStillCommand;
import frc.robot.Commands.PickUpCommand;
import frc.robot.Commands.HighShootTimed;
import frc.robot.Subsystems.DriveSubsystem;
import frc.robot.Subsystems.EgressSubsystem;
import frc.robot.Subsystems.FlapSubsystem;
import frc.robot.Subsystems.IntakeModule;
import frc.robot.Subsystems.SweeperWheelsSubsystem;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class BLUESpeakerRightNote extends SequentialCommandGroup     {
    
    public BLUESpeakerRightNote(DriveSubsystem drive, EgressSubsystem shoot, IntakeModule conveyorBelt, SweeperWheelsSubsystem sweepers, FlapSubsystem flap){
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
          new DriveCommandDistance(drive, -0.5, 0, 0, 2.3), 
          new WaitCommand(0.3),
          new MotorsStillCommand(shoot, conveyorBelt, sweepers,flap),
          new WaitCommand(0.1),
          new ResetWheelPositionCommand(drive),        
          new DriveCommandDistance(drive, 0.5, 0, 0, 2.3),         
          new WaitCommand(.1), 
          new DriveTimed(drive, 0, 0, 0.3, 0.52),
          new WaitCommand(.1),
          new DriveTimed(drive, .2, 0, 0, .43),
          new WaitCommand(0.1),
          new HighShootTimed(shoot, conveyorBelt, sweepers,flap, 0.6),
          new WaitCommand(0.1),
          new DriveTimed(drive, 0, 0, -0.3, 0.30),
          new WaitCommand(0.1),
          new ResetWheelPositionCommand(drive),
          new DriveCommandDistance(drive, -0.5, 0, 0, 2.2)
             
            );
    }

    

}
