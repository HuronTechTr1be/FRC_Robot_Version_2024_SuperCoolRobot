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

public class BOTHRightWaitRetreat extends SequentialCommandGroup {

  public BOTHRightWaitRetreat(DriveSubsystem drive, EgressSubsystem shoot, IntakeModule conveyorBelt,
      SweeperWheelsSubsystem sweepers, FlapSubsystem flap) {
    super(

        new HighShootTimed(shoot, conveyorBelt, sweepers, flap, 0.6),

        new WaitCommand(2),

        new DriveTimed(drive, -0.2, 0, 0, 0.4),

        new ResetWheelPositionCommand(drive),

        new WaitCommand(0.1),

        new DriveTimed(drive, 0, 0, -0.2, 0.7),

        new WaitCommand(0.1),

        new ResetWheelPositionCommand(drive),

        new DriveTimed(drive, -0.5, 0, 0, 1.4)
        
    );
  }

}
