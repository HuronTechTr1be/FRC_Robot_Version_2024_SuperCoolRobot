package frc.robot.Subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkLimitSwitch;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ArmConstants;

public class ClawBasic {

  private CANSparkMax arm;
  private RelativeEncoder m_RelativeEncoder;
  private double m_PointRaised = 117;

  private String m_armSide;
  private SparkLimitSwitch m_LimitSwitch;

  public ClawBasic(int deviceId, String armSide) {

    arm = new CANSparkMax(deviceId, MotorType.kBrushless);
    m_RelativeEncoder = arm.getEncoder();
    m_LimitSwitch = arm.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);

    m_armSide = armSide;

  }

  public void setRampRate(int rampRate) {

    arm.setOpenLoopRampRate(rampRate);

  }

  public void ArmsUp(double speed) {

    if (isRaised())
      arm.set(0);
    else
      arm.set((speed));
  }

  public void ArmsDown() {

    if (isLowered())
      arm.set(0);
    else
      arm.set(ArmConstants.k_initArmSpeedDown);

  }

  public void adjustedArmsDownInit(double speed) {

    arm.set(speed);

  }

  public void ArmsDownInit() {

    arm.set(ArmConstants.k_initArmSpeedRoboInit);

  }

  public void ArmsStill() {

    arm.set(0);

  }

  public boolean isRaised() {

    return Math.abs(m_PointRaised - m_RelativeEncoder.getPosition()) <= 5;

  }

  public boolean isLowered() {

    if (m_LimitSwitch.isPressed()) {
      return true;
    } else {
      return false;
    }

  }

  public double getCurrent() {
    double ArmCurrent = arm.getOutputCurrent();
    SmartDashboard.putNumber(m_armSide + "ArmCurrent", ArmCurrent);
    return ArmCurrent;

  }

  public void finshZero() {

    m_RelativeEncoder.setPosition(0);
    arm.setOpenLoopRampRate(0);
    arm.burnFlash();

  }

  public double getPosition() {
    return m_RelativeEncoder.getPosition();
  }

  public void periodic() {

    if (arm.getDeviceId() == 21) {

      if (isLowered()) {
        m_RelativeEncoder.setPosition(0);
      }

    } else if (arm.getDeviceId() == 22) {

      if (isLowered()) {
        m_RelativeEncoder.setPosition(0);
      }

    }

  }

}
