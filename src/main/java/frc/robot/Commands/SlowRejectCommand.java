package frc.robot.Commands;
import frc.robot.Subsystems.EgressSubsystem;
import frc.robot.Subsystems.IntakeModule;
import frc.robot.Subsystems.SweeperWheelsSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

  public class SlowRejectCommand extends Command {
  // The subsystem the command runs on
  private final EgressSubsystem m_Shoot;
  private final IntakeModule m_conveyorBelt;
  private final SweeperWheelsSubsystem m_Sweepers;

  public SlowRejectCommand(EgressSubsystem subsystem, IntakeModule subsystem2, SweeperWheelsSubsystem subsystem3) {
    m_Shoot = subsystem;
    m_conveyorBelt = subsystem2;
    m_Sweepers = subsystem3;
    addRequirements(m_Shoot, m_conveyorBelt, m_Sweepers);
  }

  @Override
  public void initialize() {
    m_Shoot.Reject(-0.2);
    m_conveyorBelt.Reject(-0.2);
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
