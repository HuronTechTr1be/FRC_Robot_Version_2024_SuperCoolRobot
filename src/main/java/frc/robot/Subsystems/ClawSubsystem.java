
package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ArmConstants;
import com.revrobotics.SparkLimitSwitch;



public class ClawSubsystem extends SubsystemBase {
  
private ClawBasic m_armLeft = new ClawBasic(21, "left");
private ClawBasic m_armRight = new ClawBasic(22, "right");

//boolean movingUp = false;

public void armSetZero(){
 

  SmartDashboard.putNumber("inside armSetZero void",1);
  double LeftArmCurrent = m_armLeft.getCurrent();
  double RightArmCurrent = m_armRight.getCurrent();
  int x = 0;
  double armDownInitFactor = SmartDashboard.getNumber("Arm Down Init Factor:", ArmConstants.k_initArmSpeedRoboInit);
  SmartDashboard.setDefaultNumber("Arm Down Init Factor:", armDownInitFactor);
  m_armLeft.setRampRate(3);
  m_armRight.setRampRate(3);

  
  if(Math.abs(armDownInitFactor)>1){
    armDownInitFactor = 0;
  }

  if(armDownInitFactor==ArmConstants.k_initArmSpeedRoboInit){
  m_armRight.ArmsDownInit();
  m_armLeft.ArmsDownInit();

  }
else{
  m_armLeft.adjustedArmsDownInit(armDownInitFactor);
  m_armRight.adjustedArmsDownInit(armDownInitFactor);
}
    //waitCommand.execute();

  while((LeftArmCurrent<20 || RightArmCurrent<20) && x<100000){
      
    LeftArmCurrent = m_armLeft.getCurrent();
    RightArmCurrent = m_armRight.getCurrent();

    if (LeftArmCurrent>=20)
      m_armLeft.ArmsStill();
      
    if (RightArmCurrent>=20)
      m_armRight.ArmsStill();

    x++;
    SmartDashboard.putNumber("x", x);

     //waitCommand.execute();

  } 
  
  m_armLeft.ArmsStill();
  m_armRight.ArmsStill();

  m_armLeft.finshZero();
  m_armRight.finshZero();
}



  public void raiseArmsPeriodic(){
    // double leftPosition = m_armLeft.getPosition();
    // double rightPosition = m_armRight.getPosition();
      if(m_armLeft.isRaised()){
        m_armLeft.ArmsStill();
      }
      if(m_armRight.isRaised()){
        m_armRight.ArmsStill();
      }
    }

    public boolean BothArmsRaised(){
      if(m_armLeft.isRaised() && m_armRight.isRaised()){
        return true;
      }
      else{
        return false;
      }
      }
    
    

  public void periodic(){

    m_armRight.periodic();
    m_armLeft.periodic();
    
  }

  public void LeftArmUp(double speed) {
    
    m_armLeft.ArmsUp(speed);

  }

  public void RightArmUp(double speed) {
  
    m_armRight.ArmsUp(speed);

  }

  public void LeftArmDown() {
  
    m_armLeft.ArmsDown();

  }

  public void RightArmDown() {
  
    m_armRight.ArmsDown();

  }

  public void LeftArmStill() {
  
    m_armLeft.ArmsStill();

  }


  public void RightArmStill() {
  
    m_armRight.ArmsStill();

  }


}
