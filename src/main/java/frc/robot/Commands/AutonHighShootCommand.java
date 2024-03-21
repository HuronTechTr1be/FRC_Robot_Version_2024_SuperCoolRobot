package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.EgressSubsystem;
import frc.robot.Subsystems.IntakeModule;

public class AutonHighShootCommand extends Command {
  
  private final EgressSubsystem m_Shoot;
  private final IntakeModule m_conveyorBelt;

  public AutonHighShootCommand(EgressSubsystem subsystem, IntakeModule subsystem2) {
    m_Shoot = subsystem;
    m_conveyorBelt = subsystem2;
    addRequirements(m_Shoot, m_conveyorBelt);
  }

  @Override
  public void initialize() {
    m_Shoot.AutonHighShoot();
    m_conveyorBelt.HighShoot();
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
