
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



public class ClawSubsystem extends SubsystemBase {

  private CANSparkMax arm;
  private RelativeEncoder m_RelativeEncoder;
  private double m_PointRaised = 145;
  private double m_PointLowered = 0; 
  //private double m_maxLeftCurrent = 0;
  //private double m_maxRightCurrent = 0;
  private static WaitCommand waitCommand = new WaitCommand(10);


  public ClawSubsystem(int deviceId) {

    arm = new CANSparkMax(deviceId,MotorType.kBrushless);
    m_RelativeEncoder = arm.getEncoder();

  }

  public void armSetZero(){
 
    // when the robot turns on, the robot will lower until it hits zero - when the current skyrockets

    arm.setOpenLoopRampRate(1); // slowing the acceleration rate during the while loop
    double ArmCurrent = arm.getOutputCurrent();
    SmartDashboard.putNumber("ArmCurrent",ArmCurrent);
    int x = 0; 
    UppyDownyArmsDownInit();
    waitCommand.execute();

      while(ArmCurrent<20 && x<100000000){
        
       SmartDashboard.putNumber("RightArmCurrent",arm.getOutputCurrent());
       SmartDashboard.putNumber("LeftArmCurrent",arm.getOutputCurrent());
       ArmCurrent = arm.getOutputCurrent();
       x++;
       SmartDashboard.putNumber("x", x);

     } 
    
      UppyDownyArmsStill(); 
      m_RelativeEncoder.setPosition(0);
      arm.setOpenLoopRampRate(0); // sets acceleration back to normal
      // Idk if we need this - arm.burnFlash();

  }

  public void UppyDownyArmsUp(double speed) {

    if (isRaised())
      arm.set(0);
    else
      arm.set((speed));
  }

  public void UppyDownyArmsDown() {

    if (isLowered())
      arm.set(0);
    else
      arm.set(ArmConstants.k_initArmSpeedDown);

  }
  
 public void UppyDownyArmsDownInit() {

      arm.set(ArmConstants.k_initArmSpeedRoboInit);

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

  //   if(arm.getDeviceId()==21){
  //     SmartDashboard.putNumber("LeftArmEncoder",m_RelativeEncoder.getPosition());
  //     SmartDashboard.putNumber("LefttArmCurrent",arm.getOutputCurrent());
  //     if (arm.getOutputCurrent()>m_maxLeftCurrent){
  //       m_maxLeftCurrent = arm.getOutputCurrent();
  //   }
  //   SmartDashboard.putNumber("maxLeftCurrent", m_maxLeftCurrent);
  // }
  //   else if(arm.getDeviceId()==22){
  //     SmartDashboard.putNumber("RightArmEncoder",m_RelativeEncoder.getPosition());
  //     SmartDashboard.putNumber("RightArmCurrent",arm.getOutputCurrent());
  //     if (arm.getOutputCurrent()>m_maxRightCurrent){
  //       m_maxRightCurrent = arm.getOutputCurrent();
  //   }
  //   SmartDashboard.putNumber("maxRightCurrent", m_maxRightCurrent);

  //   }

  }

}
