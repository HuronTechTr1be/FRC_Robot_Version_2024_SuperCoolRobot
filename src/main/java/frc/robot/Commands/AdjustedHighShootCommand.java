package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.EgressSubsystem;
import frc.robot.Subsystems.IntakeModule;

public class AdjustedHighShootCommand extends Command {
  
  private final EgressSubsystem m_Shoot;
  private final IntakeModule m_conveyorBelt;
  private final double powerLevel;

  public AdjustedHighShootCommand(EgressSubsystem subsystem, IntakeModule subsystem2, double power) {
    m_Shoot = subsystem;
    m_conveyorBelt = subsystem2;
    powerLevel = power;
    addRequirements(m_Shoot, m_conveyorBelt);
  }

  @Override
  public void initialize() {
    m_Shoot.AdjustedHighShoot(powerLevel);
    m_conveyorBelt.HighShoot();
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
