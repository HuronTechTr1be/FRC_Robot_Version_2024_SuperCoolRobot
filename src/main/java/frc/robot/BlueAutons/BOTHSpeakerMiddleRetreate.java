package frc.robot.BlueAutons;

import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Commands.HighShootTimed;
import frc.robot.Commands.MotorsStillCommand;
import frc.robot.Commands.ResetWheelPositionCommand;
import frc.robot.Subsystems.DriveSubsystem;
import frc.robot.Subsystems.EgressSubsystem;
import frc.robot.Subsystems.FlapSubsystem;
import frc.robot.Subsystems.IntakeModule;
import frc.robot.Subsystems.SweeperWheelsSubsystem;

public class BOTHSpeakerMiddleRetreate extends SequentialCommandGroup {

  public BOTHSpeakerMiddleRetreate(DriveSubsystem drive, EgressSubsystem shoot, IntakeModule conveyorBelt,
      SweeperWheelsSubsystem sweepers, FlapSubsystem flap) {
    super(
        new HighShootTimed(shoot, conveyorBelt, sweepers, flap, 0.8),
        new WaitCommand(0.5),
        new MotorsStillCommand(shoot, conveyorBelt, sweepers, flap),
        new ResetWheelPositionCommand(drive),
        new RunCommand(() -> drive.drive(-0.3, 0, 0, false, true), drive).until(() -> drive.checkEncoder(.8))

        );
  }

}
