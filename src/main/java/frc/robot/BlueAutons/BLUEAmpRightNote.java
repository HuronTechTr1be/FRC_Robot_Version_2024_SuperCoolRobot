package frc.robot.BlueAutons;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Commands.DriveCommandDistance;
import frc.robot.Commands.DriveTimed;
import frc.robot.Commands.FlapDownCommand;
import frc.robot.Commands.FlapUpCommand;
import frc.robot.Commands.LowShootTimed;
import frc.robot.Commands.MotorsStillCommand;
import frc.robot.Commands.PickUpCommand;
import frc.robot.Commands.ResetWheelPositionCommand;
import frc.robot.Subsystems.DriveSubsystem;
import frc.robot.Subsystems.EgressSubsystem;
import frc.robot.Subsystems.FlapSubsystem;
import frc.robot.Subsystems.IntakeModule;
import frc.robot.Subsystems.SweeperWheelsSubsystem;

public class BLUEAmpRightNote extends SequentialCommandGroup {

  public BLUEAmpRightNote(DriveSubsystem drive, EgressSubsystem shoot, IntakeModule conveyorBelt,
      SweeperWheelsSubsystem sweepers, FlapSubsystem flap) {
    addCommands(

        new FlapUpCommand(flap),
        new WaitCommand(0.1),
        new LowShootTimed(shoot, conveyorBelt, sweepers, flap, 0.9, 0.3),
        new FlapDownCommand(flap),

        new ResetWheelPositionCommand(drive),
        new DriveCommandDistance(drive, -0.5, 0, 0, 0.6),

        new WaitCommand(0.3),

        new DriveTimed(drive, 0, 0, 0.4, 0.5),

        new WaitCommand(0.3),
        new PickUpCommand(shoot, conveyorBelt, sweepers),

        new ResetWheelPositionCommand(drive),
        new DriveCommandDistance(drive, -0.5, 0, 0, -0.8),

        new WaitCommand(0.2),
        new MotorsStillCommand(shoot, conveyorBelt, sweepers, flap),
        new WaitCommand(0.3),

        new ResetWheelPositionCommand(drive),
        new DriveCommandDistance(drive, 0.5, 0, 0, 1),

        new WaitCommand(0.3),

        new DriveTimed(drive, 0, 0, -0.4, 0.45),

        new WaitCommand(0.3),

        new ResetWheelPositionCommand(drive),
        new DriveCommandDistance(drive, 0.5, 0, 0, 0.8),

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
