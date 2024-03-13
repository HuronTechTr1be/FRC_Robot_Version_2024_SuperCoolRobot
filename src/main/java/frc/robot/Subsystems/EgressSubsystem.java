package frc.robot.Subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class EgressSubsystem extends SubsystemBase {

  private EgressBasic m_rightTop = new EgressBasic(42);
  private EgressBasic m_rightBottom = new EgressBasic(41);

  private EgressBasic m_leftTop = new EgressBasic(43);
  private EgressBasic m_leftBottom = new EgressBasic(44);

  public void PickUp() {
    m_rightTop.PickUp();
    m_rightBottom.PickUp();

    m_leftTop.PickUp();
    m_leftBottom.PickUp();

  }

  public void Reject() {
    m_rightTop.Reject();
    m_rightBottom.Reject();

    m_leftTop.Reject();
    m_leftBottom.Reject();

  }

  public void Reject(double speed) {
    m_rightBottom.Reject(speed);

    m_leftBottom.Reject(speed);

  }

  public void HighShoot() {
    m_rightTop.HighShoot();
    m_rightBottom.HighShoot();

    m_leftTop.HighShoot();
    m_leftBottom.HighShoot();

  }

  public void LowShoot() {
    m_rightTop.LowShoot();
    m_rightBottom.LowShoot();

    m_leftTop.LowShoot();
    m_leftBottom.LowShoot();

  }

  public void adjustedLowShoot(double shootPower) {
    m_rightTop.adjustedLowShoot(shootPower);
    m_rightBottom.adjustedLowShoot(shootPower);

    m_leftTop.adjustedLowShoot(shootPower);
    m_leftBottom.adjustedLowShoot(shootPower);

  }

  public void Still() {
    m_rightBottom.Still();
    m_rightTop.Still();

    m_leftBottom.Still();
    m_leftTop.Still();

  }

}
