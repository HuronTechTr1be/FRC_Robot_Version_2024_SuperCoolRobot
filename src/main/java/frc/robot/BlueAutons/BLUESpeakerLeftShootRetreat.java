package frc.robot.BlueAutons;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Commands.DriveCommandDistance;
import frc.robot.Commands.HighShootTimed;
import frc.robot.Subsystems.DriveSubsystem;
import frc.robot.Subsystems.EgressSubsystem;
import frc.robot.Subsystems.FlapSubsystem;
import frc.robot.Subsystems.IntakeModule;
import frc.robot.Subsystems.SweeperWheelsSubsystem;

public class BLUESpeakerLeftShootRetreat extends SequentialCommandGroup {

  public BLUESpeakerLeftShootRetreat(DriveSubsystem drive, EgressSubsystem shoot, IntakeModule conveyorBelt,
      SweeperWheelsSubsystem sweepers, FlapSubsystem flap) {
    addCommands(
        new HighShootTimed(shoot, conveyorBelt, sweepers, flap, 0.3),
        new WaitCommand(0.5),
        new DriveCommandDistance(drive, -0.5, 0, 0, 2));
  }

}
