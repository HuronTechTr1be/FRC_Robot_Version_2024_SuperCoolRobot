
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

  private CANSparkMax arm;
  private RelativeEncoder m_RelativeEncoder;
  private SparkLimitSwitch m_LimitSwitch;
  private double m_PointRaised = 145;
  private double m_PointLowered = 0; 
  // private double m_maxLeftCurrent = 0;
  // private double m_maxRightCurrent = 0;
  private static WaitCommand waitCommand = new WaitCommand(10);

  

  public ClawSubsystem(int deviceId) {

    arm = new CANSparkMax(deviceId,MotorType.kBrushless);
    m_RelativeEncoder = arm.getEncoder();
    m_LimitSwitch = arm.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);

  }

  

  public void armSetZero(){
 
    waitCommand.initialize();

    SmartDashboard.putNumber("inside armSetZero void",1);
    double ArmCurrent = arm.getOutputCurrent();
    SmartDashboard.putNumber("ArmCurrent",ArmCurrent);
    int x = 0;
    double armDownInitFactor = SmartDashboard.getNumber("Bottom High Shoot Factor:", ArmConstants.k_initArmSpeedRoboInit);
    SmartDashboard.setDefaultNumber("Bottom High Shoot Factor:", armDownInitFactor);
    if(Math.abs(armDownInitFactor)>1){
      armDownInitFactor = 0;
    }
    if(armDownInitFactor==ArmConstants.k_initArmSpeedRoboInit){
    UppyDownyArmsDownInit();
    }
  else{
    adjustedUppyDownyArmsDownInit(armDownInitFactor);
  }
    waitCommand.execute();
    //double m_maxRight = 0;
    //double m_maxLeft = 0;

      while(ArmCurrent<20 && x<50000){
        
       arm.setOpenLoopRampRate(3);
       SmartDashboard.putNumber("RightArmCurrent",arm.getOutputCurrent());
       SmartDashboard.putNumber("LeftArmCurrent",arm.getOutputCurrent());
       ArmCurrent = arm.getOutputCurrent();
       x++;
       SmartDashboard.putNumber("x", x);

    //    if (arm.getOutputCurrent()>=m_maxRight){
    //     m_maxRight = arm.getOutputCurrent();
    //     SmartDashboard.putNumber("MaxRightArmCurrentWhileLoop",m_maxRight);
    // }

      waitCommand.execute();

     } 
    
      UppyDownyArmsStill(); 
      m_RelativeEncoder.setPosition(0);
      arm.setOpenLoopRampRate(0);
      arm.burnFlash();

  }

  public void UppyDownyArmsUp(double speed) {

    if (isRaised())
      arm.set(0);
    else
      arm.set((speed));
  }

  public void UppyDownyArmsDown() {


    if (isLowered()){
      arm.set(0);
    }
      else{
      arm.set(ArmConstants.k_initArmSpeedDown);
      }
  }

 public void adjustedUppyDownyArmsDownInit(double speed){
  
  arm.set(speed);
 
  } 

 public void UppyDownyArmsDownInit() {

  arm.set(ArmConstants.k_initArmSpeedRoboInit);

  }

  public void UppyDownyArmsStill() {

    arm.set(0);

  }

  private boolean isRaised(){

    return Math.abs(m_PointRaised - m_RelativeEncoder.getPosition()) <= 5;

      
  }

  private boolean isLowered(){

    if (m_LimitSwitch.isPressed()){
      if(arm.getDeviceId()==21){
      SmartDashboard.putBoolean("Left Arm Down", true);
      }
      if(arm.getDeviceId()==22){
      SmartDashboard.putBoolean("Right Arm Down", true);
      }
      return true;
    }
    else{
      if(arm.getDeviceId()==21){
      SmartDashboard.putBoolean("Left Arm Down", false);
      }
      if(arm.getDeviceId()==22){
      SmartDashboard.putBoolean("Right Arm Down", false);
      }      
      return false;
    }

  }

  public void periodic(){
    

    if(arm.getDeviceId()==21){ 
      SmartDashboard.putNumber("LeftArmEncoder",m_RelativeEncoder.getPosition());
      SmartDashboard.putNumber("LefttArmCurrent",arm.getOutputCurrent());
      if (m_LimitSwitch.isPressed()){
      SmartDashboard.putBoolean("LeftisPressed", true);
    }
    else{
      SmartDashboard.putBoolean("LeftisPressed", false);
    }
    //   if (arm.getOutputCurrent()>m_maxLeftCurrent){
    //     m_maxLeftCurrent = arm.getOutputCurrent();
    // }
    // SmartDashboard.putNumber("maxLeftCurrent", m_maxLeftCurrent);
  }
    else if(arm.getDeviceId()==22){
      SmartDashboard.putNumber("RightArmEncoder",m_RelativeEncoder.getPosition());
      SmartDashboard.putNumber("RightArmCurrent",arm.getOutputCurrent());
      if (m_LimitSwitch.isPressed()){
      SmartDashboard.putBoolean("RightisPressed", true);
    }
      else{
      SmartDashboard.putBoolean("RightisPressed", false);
      }
      // if (arm.getOutputCurrent()>m_maxRightCurrent){
      //   m_maxRightCurrent = arm.getOutputCurrent();
      //}
    // SmartDashboard.putNumber("maxRightCurrent", m_maxRightCurrent);

    }

  }

}
