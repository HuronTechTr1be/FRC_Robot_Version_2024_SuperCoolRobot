package frc.robot.Commands;
import frc.robot.Subsystems.FlapSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

  public class FlapStillCommand extends Command {
  // The subsystem the command runs on
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
