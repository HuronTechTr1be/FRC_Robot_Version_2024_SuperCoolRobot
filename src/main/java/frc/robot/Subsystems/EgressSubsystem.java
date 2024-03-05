package frc.robot.Subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class EgressSubsystem extends SubsystemBase {

  private EgressBasic m_top = new EgressBasic(42);
  private EgressBasic m_bottom = new EgressBasic(41);

  public void PickUp() {
    m_top.PickUp();
    m_bottom.PickUp();
  }

  public void Reject() {
    m_top.Reject();
    m_bottom.Reject();
  }

  public void Reject(double speed) {
    m_bottom.Reject(speed);
  }

  public void HighShoot() {
    m_top.HighShoot();
    m_bottom.HighShoot();
  }

  public void LowShoot() {
    m_top.LowShoot();
    m_bottom.LowShoot();
  }

  public void adjustedLowShoot(double shootPower) {
    m_top.adjustedLowShoot(shootPower);
    m_bottom.adjustedLowShoot(shootPower);
  }

  public void Still() {
    m_bottom.Still();
    m_top.Still();
  }

}
