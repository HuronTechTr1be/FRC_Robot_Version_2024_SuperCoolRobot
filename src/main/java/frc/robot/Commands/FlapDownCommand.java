package frc.robot.Commands;

import frc.robot.Subsystems.FlapSubsystem; 
import edu.wpi.first.wpilibj2.command.Command;

  public class FlapDownCommand extends Command {
  // The subsystem the command runs on
  private final FlapSubsystem m_flap;

  public FlapDownCommand(FlapSubsystem flap) {
    m_flap = flap;
    addRequirements(m_flap);
  }

  @Override
  public void initialize() {
    m_flap.FlapDown();
    while(m_flap.getFlapEncoder()>(-15)){

    }
    
    m_flap.FlapStill();
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
