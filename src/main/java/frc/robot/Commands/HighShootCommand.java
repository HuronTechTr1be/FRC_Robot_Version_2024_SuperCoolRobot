package frc.robot.Commands;
import frc.robot.Subsystems.EgressSubsystem;
import frc.robot.Subsystems.IntakeModule;
import edu.wpi.first.wpilibj2.command.Command;

  public class HighShootCommand extends Command {
  // The subsystem the command runs on
  private final EgressSubsystem m_Shoot;
  private final IntakeModule m_conveyorBelt;

  public HighShootCommand(EgressSubsystem subsystem, IntakeModule subsystem2) {
    m_Shoot = subsystem;
    m_conveyorBelt = subsystem2;
    addRequirements(m_Shoot, m_conveyorBelt);
  }

  @Override
  public void initialize() {
    m_Shoot.HighShoot();
    m_conveyorBelt.HighShoot();
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
