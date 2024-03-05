package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.EgressSubsystem;
import frc.robot.Subsystems.FlapSubsystem;
import frc.robot.Subsystems.IntakeModule;
import frc.robot.Subsystems.SweeperWheelsSubsystem;

public class MotorsStillCommand extends Command {

  private final EgressSubsystem m_Shoot;
  private final IntakeModule m_conveyorBelt;
  private final SweeperWheelsSubsystem m_Sweepers;
  private final FlapSubsystem m_flap;

  public MotorsStillCommand(EgressSubsystem subsystem, IntakeModule subsystem2, SweeperWheelsSubsystem subsystem3,
      FlapSubsystem subsystem4) {
    m_Shoot = subsystem;
    m_conveyorBelt = subsystem2;
    m_Sweepers = subsystem3;
    m_flap = subsystem4;
    addRequirements(m_Shoot, m_conveyorBelt, m_Sweepers, m_flap);
  }

  @Override
  public void initialize() {
    m_Shoot.Still();
    m_conveyorBelt.Still();
    m_Sweepers.Still();
    m_flap.FlapStill();
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
