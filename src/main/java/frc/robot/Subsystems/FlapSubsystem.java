package frc.robot.Subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkLimitSwitch;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FlapConstants;

public class FlapSubsystem extends SubsystemBase {

  private CANSparkMax flap;
  private RelativeEncoder m_RelativeEncoder;
  private SparkLimitSwitch m_LimitSwitch;
  private double m_PointLowered = -22;

  public FlapSubsystem(int deviceId) {

    flap = new CANSparkMax(deviceId, MotorType.kBrushless);
    m_RelativeEncoder = flap.getEncoder();
    m_LimitSwitch = flap.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);

  }

  public double getFlapEncoder() {
    return m_RelativeEncoder.getPosition();
  }

  public void flapEncoderZero() {
    m_RelativeEncoder.setPosition(0);
  }

  public void flapSetZero() {

    int x = 0;
    flap.setOpenLoopRampRate(1.0);
    UppyDownyFlapUpInit();

    x = 0;
    while (!(isRaised()) && x < 100000) {

      x++;
      SmartDashboard.putNumber("x", 27);
      SmartDashboard.putNumber("FlapCurrent", flap.getOutputCurrent());
      SmartDashboard.putNumber("FlapEncoder", m_RelativeEncoder.getPosition());

    }
    FlapStill();
    m_RelativeEncoder.setPosition(0);
    x = 0;
    FlapDown();
    while (!(isLowered()) && x < 1000) {
      x++;
      SmartDashboard.putNumber("FlapCurrent", flap.getOutputCurrent());
      SmartDashboard.putNumber("FlapEncoder", m_RelativeEncoder.getPosition());
    }
    if (m_LimitSwitch.isPressed()) {
      UppyDownyFlapUpInit();
      while (!(isRaised()) && x < 100000) {

        x++;
        SmartDashboard.putNumber("x", x);
        SmartDashboard.putNumber("FlapCurrent", flap.getOutputCurrent());
        SmartDashboard.putNumber("FlapEncoder", m_RelativeEncoder.getPosition());

      }
      FlapStill();
      m_RelativeEncoder.setPosition(0);
      x = 0;
      FlapDown();
      while (!(isLowered()) && x < 100000) {
        x++;
        SmartDashboard.putNumber("FlapCurrent", flap.getOutputCurrent());
      }
    }
    x = 0;
    while (!(isLowered() && x < 100000)) {
      x++;
    }
    FlapStill();
    flap.setOpenLoopRampRate(0);

  }

  public void FlapUp(double speed) {

    if (isRaised()) {
      flap.set(0);
      m_RelativeEncoder.setPosition(0);
    } else {
      flap.set((speed));
    }
  }

  public void FlapDown() {
    if (isLowered()) {
      flap.set(0);
      SmartDashboard.putString("FlapDown", "IsLowered");
    } else {
      SmartDashboard.putString("FlapDown", "GoingDown");
      flap.set(FlapConstants.k_FlapSpeedDown);
    }
  }

  public void UppyDownyFlapUpInit() {
    flap.set(FlapConstants.k_initFlapSpeedRoboInit);

  }

  public void FlapStill() {

    flap.set(0);
    if(isRaised()){
      m_RelativeEncoder.setPosition(0);
    }
  }

  public boolean isRaised() {

    return m_LimitSwitch.isPressed();

  }

  public boolean isLowered() {

    return ((m_PointLowered - m_RelativeEncoder.getPosition()) >= 2);

  }

  public void periodic() {

    SmartDashboard.putNumber("FlapEncoder", m_RelativeEncoder.getPosition());
    SmartDashboard.putNumber("FlapCurrent", flap.getOutputCurrent());

  }

}
