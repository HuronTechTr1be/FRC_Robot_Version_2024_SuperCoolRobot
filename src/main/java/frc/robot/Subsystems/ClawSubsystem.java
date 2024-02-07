
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
  private double m_PointRaised = 145;
  private double m_PointLowered = 0; 
  private double m_maxLeftCurrent = 0;
  private double m_maxRightCurrent = 0;


  public ClawSubsystem(int deviceId) {

    arm = new CANSparkMax(deviceId,MotorType.kBrushless);
    m_RelativeEncoder = arm.getEncoder();

  }

  public void armSetZero(){
 
    SmartDashboard.putNumber("im here",1);
    double current = arm.getOutputCurrent();
    SmartDashboard.putNumber("current",current);
      while(current<20){
 
       SmartDashboard.putNumber("RightArmCurrent",arm.getOutputCurrent());
       SmartDashboard.putNumber("LeftArmCurrent",arm.getOutputCurrent());
       UppyDownyArmsDown();

        SmartDashboard.putNumber("still here",2);
        current = arm.getOutputCurrent();
     } 
    
      UppyDownyArmsStill(); 
      m_RelativeEncoder.setPosition(0);

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

    return Math.abs(m_PointRaised - m_RelativeEncoder.getPosition()) <= 10;

  }

  private boolean isLowered(){

    return Math.abs(m_PointLowered - m_RelativeEncoder.getPosition()) <= 10;

  }

  public void periodic(){

    if(arm.getDeviceId()==21){
      SmartDashboard.putNumber("LeftArmEncoder",m_RelativeEncoder.getPosition());
      SmartDashboard.putNumber("LefttArmCurrent",arm.getOutputCurrent());
      if (arm.getOutputCurrent()>m_maxLeftCurrent){
        m_maxLeftCurrent = arm.getOutputCurrent();
    }
    SmartDashboard.putNumber("maxLeftCurrent", m_maxLeftCurrent);
  }
    else if(arm.getDeviceId()==22){
      SmartDashboard.putNumber("RightArmEncoder",m_RelativeEncoder.getPosition());
      SmartDashboard.putNumber("RightArmCurrent",arm.getOutputCurrent());
      if (arm.getOutputCurrent()>m_maxRightCurrent){
        m_maxRightCurrent = arm.getOutputCurrent();
    }
    SmartDashboard.putNumber("maxRightCurrent", m_maxRightCurrent);

    }

  }

}
