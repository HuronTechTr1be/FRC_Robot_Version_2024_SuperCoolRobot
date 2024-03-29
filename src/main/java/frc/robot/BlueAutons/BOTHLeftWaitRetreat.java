package frc.robot.BlueAutons;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Commands.DriveTimed;
import frc.robot.Commands.HighShootTimed;
import frc.robot.Commands.ResetWheelPositionCommand;
import frc.robot.Subsystems.DriveSubsystem;
import frc.robot.Subsystems.EgressSubsystem;
import frc.robot.Subsystems.FlapSubsystem;
import frc.robot.Subsystems.IntakeModule;
import frc.robot.Subsystems.SweeperWheelsSubsystem;

public class BOTHLeftWaitRetreat extends SequentialCommandGroup {

  public BOTHLeftWaitRetreat(DriveSubsystem drive, EgressSubsystem shoot, IntakeModule conveyorBelt,
      SweeperWheelsSubsystem sweepers, FlapSubsystem flap) {
    super(

        new HighShootTimed(shoot, conveyorBelt, sweepers, flap, 0.6),

        new WaitCommand(2),

        new DriveTimed(drive, -0.2, 0, 0, 0.5),

        new DriveTimed(drive, 0, 0, 0.6, 7),

        new WaitCommand(0.1),

        new ResetWheelPositionCommand(drive),

        new DriveTimed(drive, -0.5, 0, 0, 1.4)
        
    );
  }

}
