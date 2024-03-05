package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Subsystems.DriveSubsystem;

public class DriveTimed extends SequentialCommandGroup {

    public DriveTimed(DriveSubsystem subsystem, double XSpeed, double YSpeed, double Rot, double Time) {
        addCommands(
                new DriveCommand(subsystem, XSpeed, YSpeed, Rot),
                new WaitCommand(Time),
                new DriveCommand(subsystem, 0, 0, 0));
    }
}
