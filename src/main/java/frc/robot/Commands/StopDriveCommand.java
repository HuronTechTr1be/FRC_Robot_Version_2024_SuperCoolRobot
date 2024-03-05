package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.DriveSubsystem;

public class StopDriveCommand extends Command {

  private final DriveSubsystem m_Drive;

  public StopDriveCommand(DriveSubsystem subsystem) {
    m_Drive = subsystem;
    addRequirements(m_Drive);
  }

  @Override
  public void initialize() {
    m_Drive.drive(0, 0, 0, false, true);
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
