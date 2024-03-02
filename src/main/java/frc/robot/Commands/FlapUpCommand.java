package frc.robot.Commands;
import frc.robot.Subsystems.FlapSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

  public class FlapUpCommand extends Command {
  // The subsystem the command runs on
  private final FlapSubsystem m_flap;

  public FlapUpCommand(FlapSubsystem flap) {
    m_flap = flap;
    addRequirements(m_flap);
  }

  @Override
  public void initialize() {
    m_flap.FlapUp(0.3);
    while(m_flap.isRaised()==false){

    }
    m_flap.flapEncoderZero();
    m_flap.FlapStill();

  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
