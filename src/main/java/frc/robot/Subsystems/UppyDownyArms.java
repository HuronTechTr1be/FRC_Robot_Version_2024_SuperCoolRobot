package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ArmConstants;



public class UppyDownyArms {

  private CANSparkMax arm;
  private RelativeEncoder m_RelativeEncoder;
  private double m_PointRaised = 145;
  private double m_PointLowered = 0; 

  private String m_armSide;
  private static WaitCommand waitCommand = new WaitCommand(10);


  public UppyDownyArms(int deviceId, String armSide) {

    arm = new CANSparkMax(deviceId,MotorType.kBrushless);
    m_RelativeEncoder = arm.getEncoder();

    m_armSide = armSide;

  }

  public void setRampRate(int rampRate){

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

 public void adjustedArmsDownInit(double speed){
  
  arm.set(speed);
 
  } 

 public void ArmsDownInit() {

  arm.set(ArmConstants.k_initArmSpeedRoboInit);

  }

  public void ArmsStill() {

    arm.set(0);

  }

  private boolean isRaised(){

    return Math.abs(m_PointRaised - m_RelativeEncoder.getPosition()) <= 5;

      
  }

  private boolean isLowered(){

    return Math.abs(m_PointLowered - m_RelativeEncoder.getPosition()) <= 5;

  }

  public double getCurrent(){
    double ArmCurrent=arm.getOutputCurrent();
    SmartDashboard.putNumber(m_armSide + "ArmCurrent",ArmCurrent);
    return ArmCurrent;

  }

  public void finshZero(){

    m_RelativeEncoder.setPosition(0);
    arm.setOpenLoopRampRate(0);
    arm.burnFlash();
     
  }

  public void periodic(){

    if(arm.getDeviceId()==21){ 
      SmartDashboard.putNumber("LeftArmEncoder",m_RelativeEncoder.getPosition());
      SmartDashboard.putNumber("LeftArmCurrentPeriodic",arm.getOutputCurrent());
    //   if (arm.getOutputCurrent()>m_maxLeftCurrent){
    //     m_maxLeftCurrent = arm.getOutputCurrent();
    // }
    // SmartDashboard.putNumber("maxLeftCurrent", m_maxLeftCurrent);
  }
    else if(arm.getDeviceId()==22){
      SmartDashboard.putNumber("RightArmEncoder",m_RelativeEncoder.getPosition());
      SmartDashboard.putNumber("RightArmCurrentPeriodic",arm.getOutputCurrent());
      // if (arm.getOutputCurrent()>m_maxRightCurrent){
      //   m_maxRightCurrent = arm.getOutputCurrent();
      //}
    // SmartDashboard.putNumber("maxRightCurrent", m_maxRightCurrent);

    }

  }

}
