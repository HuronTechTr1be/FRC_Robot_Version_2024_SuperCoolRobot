package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.DriveSubsystem;

public class ResetWheelPositionCommand extends Command {

  private final DriveSubsystem m_Drive;

  public ResetWheelPositionCommand(DriveSubsystem subsystem) {

    m_Drive = subsystem;
    addRequirements(m_Drive);

  }

  @Override
  public void initialize() {
    m_Drive.resetFrontRightEncoder();
  }

  @Override
  public boolean isFinished() {
    return true;
  }

}
