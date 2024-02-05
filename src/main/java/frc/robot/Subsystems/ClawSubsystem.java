
package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;


public class ClawSubsystem extends SubsystemBase {

  private CANSparkMax arm;
  private RelativeEncoder m_RelativeEncoder;
  private double m_setPointRaised = 145;
    private double m_setPointLowered = 0; 

  public ClawSubsystem(int deviceId) {

    arm = new CANSparkMax(deviceId,MotorType.kBrushless);
    m_RelativeEncoder = arm.getEncoder();

  }

  public void UppyDownyArmsUp(double speed) {

    if (isRaised())
      arm.set(0);
    else
      arm.set(speed);
  }

  public void UppyDownyArmsDown() {

    if (isLowered())
      arm.set(0);
    else
      arm.set(-1);

  }
  
  public void UppyDownyArmsStill() {

    arm.set(0);

  }

  private boolean isRaised(){

    return Math.abs(m_setPointRaised - m_RelativeEncoder.getPosition()) <= 10;

  }

  private boolean isLowered(){

    return Math.abs(m_setPointLowered - m_RelativeEncoder.getPosition()) <= 10;

  }

  public void periodic(){

    if(arm.getDeviceId()==21){
      SmartDashboard.putNumber("LeftArmEncoder",m_RelativeEncoder.getPosition());
    }
    else if(arm.getDeviceId()==22){
      SmartDashboard.putNumber("RightArmEncoder",m_RelativeEncoder.getPosition());
    }
    
  }

}
