package frc.robot.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Subsystems.EgressSubsystem;
import frc.robot.Subsystems.FlapSubsystem;
import frc.robot.Subsystems.IntakeModule;
import frc.robot.Subsystems.SweeperWheelsSubsystem;

public class HighShootTimed extends SequentialCommandGroup{
    public HighShootTimed(EgressSubsystem shoot, IntakeModule conveyorBelt, SweeperWheelsSubsystem sweepers, FlapSubsystem flap, double time){
        addCommands(
            new HighShootCommand(shoot, conveyorBelt),
            new WaitCommand(time),
            new MotorsStillCommand(shoot, conveyorBelt, sweepers, flap)
        
        );
    }
}
