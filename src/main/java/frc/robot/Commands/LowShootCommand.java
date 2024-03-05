package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.EgressSubsystem;
import frc.robot.Subsystems.IntakeModule;

public class LowShootCommand extends Command {

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
