package frc.robot.Commands;
import frc.robot.Subsystems.EgressSubsystem;
import frc.robot.Subsystems.IntakeModule;
import edu.wpi.first.wpilibj2.command.Command;

  public class LowShootCommand extends Command {
  // The subsystem the command runs on
  private final EgressSubsystem m_Shoot;
  private final IntakeModule m_conveyorBelt;
  private final double shootFactor;


  public LowShootCommand(EgressSubsystem subsystem, IntakeModule subsystem2, double shootPower) {
    m_Shoot = subsystem;
    m_conveyorBelt = subsystem2;
    shootFactor = shootPower;
    addRequirements(m_Shoot, m_conveyorBelt);
  }

  @Override
  public void initialize() {
    m_Shoot.adjustedLowShoot(shootFactor);
    m_conveyorBelt.LowShoot();
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
