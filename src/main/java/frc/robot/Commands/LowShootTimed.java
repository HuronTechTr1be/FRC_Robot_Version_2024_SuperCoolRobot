package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Subsystems.DriveSubsystem;
import frc.robot.Subsystems.EgressSubsystem;
import frc.robot.Subsystems.IntakeModule;
import frc.robot.Subsystems.SweeperWheelsSubsystem;
import frc.robot.Commands.HighShootCommand;
import edu.wpi.first.wpilibj2.command.Command;

public class LowShootTimed extends SequentialCommandGroup{
    public LowShootTimed(EgressSubsystem shoot, IntakeModule conveyorBelt, SweeperWheelsSubsystem sweepers, double time){
        addCommands(
            new LowShootCommand(shoot, conveyorBelt),
            new WaitCommand(time),
            new MotorsStillCommand(shoot, conveyorBelt, sweepers)
        
        );
    }
}
