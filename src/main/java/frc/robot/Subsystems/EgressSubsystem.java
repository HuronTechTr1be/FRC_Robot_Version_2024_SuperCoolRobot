package frc.robot.Subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class EgressSubsystem extends SubsystemBase {

  private EgressBasic m_LeftTop = new EgressBasic(42);
  private EgressBasic m_LeftBottom = new EgressBasic(41);

  private EgressBasic m_RightTop = new EgressBasic(44);
  private EgressBasic m_RightBottom = new EgressBasic(43);

  public void PickUp() {
    m_RightTop.PickUp();
    m_RightBottom.PickUp();
    m_LeftTop.PickUp();
    m_LeftBottom.PickUp();
  }

  public void Reject() {
    m_RightTop.Reject();
    m_RightBottom.Reject();
    m_LeftTop.Reject();
    m_LeftBottom.Reject();
  }

  public void Reject(double speed) {
    m_RightBottom.Reject(speed);
    m_LeftBottom.Reject(speed);
  }

  public void HighShoot() {
    m_RightTop.HighShoot();
    m_RightBottom.HighShoot();
    m_LeftTop.HighShoot();
    m_LeftBottom.HighShoot();
  }

  public void LowShoot() {
    m_RightTop.LowShoot();
    m_RightBottom.LowShoot();
    m_LeftTop.LowShoot();
    m_LeftBottom.LowShoot();
  }

  public void adjustedLowShoot(double shootPower) {
    m_RightTop.adjustedLowShoot(shootPower);
    m_RightBottom.adjustedLowShoot(shootPower);
    m_LeftTop.adjustedLowShoot(shootPower);
    m_LeftBottom.adjustedLowShoot(shootPower);
  }

  public void Still() {
    m_RightBottom.Still();
    m_RightTop.Still();
    m_LeftBottom.Still();
    m_LeftTop.Still();
  }

}
