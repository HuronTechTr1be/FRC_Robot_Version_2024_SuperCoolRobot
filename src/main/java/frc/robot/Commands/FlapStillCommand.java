package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.FlapSubsystem;

public class FlapStillCommand extends Command {

  private final FlapSubsystem m_flap;

  public FlapStillCommand(FlapSubsystem flap) {
    m_flap = flap;
    addRequirements(m_flap);
  }

  @Override
  public void initialize() {

    m_flap.FlapStill();

  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
