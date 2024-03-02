package frc.robot.BlueAutons;
import frc.robot.Commands.DriveCommandDistance;
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

// MAKE SURE YOU ARE A BIT AWAY FROM THE SPEAKER

public class BOTHSpeakerMiddleNote extends SequentialCommandGroup     {
    
    public BOTHSpeakerMiddleNote(DriveSubsystem drive, EgressSubsystem shoot, IntakeModule conveyorBelt, SweeperWheelsSubsystem sweepers, FlapSubsystem flap){
        addCommands(
            new HighShootTimed(shoot, conveyorBelt, sweepers,flap, 0.3),
            new WaitCommand(0.5),
            new PickUpCommand(shoot, conveyorBelt, sweepers),
            new WaitCommand(.1), 
            new ResetWheelPositionCommand(drive),        
            new DriveCommandDistance(drive, -0.5, 0, 0, -1.7),
            new WaitCommand(0.2),
            new MotorsStillCommand(shoot, conveyorBelt, sweepers,flap),
            new WaitCommand(.1),
            new ResetWheelPositionCommand(drive),
            new DriveCommandDistance(drive, 0.5, 0, 0, 1.7),  
            new WaitCommand(.1),
            new HighShootTimed(shoot, conveyorBelt, sweepers, flap,0.3),
            new ResetWheelPositionCommand(drive),        
            new DriveCommandDistance(drive, -0.5, 0, 0, -1.5)         
            );
    }

    

}
