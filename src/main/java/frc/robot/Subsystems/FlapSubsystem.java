package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.FlapConstants;

public class FlapSubsystem extends SubsystemBase {
    
    private CANSparkMax flap;
    private RelativeEncoder m_RelativeEncoder;
    private double m_PointRaised = 145;
    private double m_PointLowered = 0; 
    private double m_maxFlapCurrent = 0;
    private static WaitCommand waitCommand = new WaitCommand(10);

    public FlapSubsystem(int deviceId){

        flap = new CANSparkMax(deviceId,MotorType.kBrushless);
        m_RelativeEncoder = flap.getEncoder();


    }

    public void flapSetZero(){
 
    waitCommand.initialize();

    SmartDashboard.putNumber("inside flapSetZero void",1);
    double FlapCurrent = flap.getOutputCurrent();
    SmartDashboard.putNumber("FlapCurrent",FlapCurrent);
    int x = 0; 
    UppyDownyFlapDownInit();
    waitCommand.execute();

      while(FlapCurrent<20 && x<50000){
 
       flap.setOpenLoopRampRate(1.0);
       SmartDashboard.putNumber("FlapCurrent",flap.getOutputCurrent());
       SmartDashboard.putNumber("iinside flapSetZero while loop",2);
       FlapCurrent = flap.getOutputCurrent();
       SmartDashboard.putNumber("still here(flap)",2323);
       x++;
       SmartDashboard.putNumber("x", x);
    
      waitCommand.execute();
      }
     
    
      UppyDownyFlapStill(); 
      m_RelativeEncoder.setPosition(0);
      flap.setOpenLoopRampRate(0);
      // Idk if we need this - arm.burnFlash();

    }
  

  public void UppyDownyFlapUp(double speed) {

    if (isRaised())
      flap.set(0);
    else
      flap.set((speed));
  }

  public void UppyDownyFlapDown() {
    if (isLowered())
      flap.set(0);
    else
      flap.set(FlapConstants.k_initFlapSpeedDown);

  }
  
 public void UppyDownyFlapDownInit() {
      flap.set(FlapConstants.k_initFlapSpeedRoboInit);

  }

  public void UppyDownyFlapStill() {

    flap.set(0);

  }

  private boolean isRaised(){

    return Math.abs(m_PointRaised - m_RelativeEncoder.getPosition()) <= 10;

  }

  private boolean isLowered(){

    return Math.abs(m_PointLowered - m_RelativeEncoder.getPosition()) <= 10;

  }

  public void periodic(){

      SmartDashboard.putNumber("FlapEncoder",m_RelativeEncoder.getPosition());
      SmartDashboard.putNumber("FlapCurrent",flap.getOutputCurrent());
      if (flap.getOutputCurrent()>m_maxFlapCurrent){
        m_maxFlapCurrent = flap.getOutputCurrent();
        SmartDashboard.putNumber("maxFlapCurrent", m_maxFlapCurrent);
      }
  }
    
    // public void Up(){

    //     flap.set(1);

    // }

    // public void Down(){

    //     flap.set(-1);

    // }

    // public void Still(){

    //     flap.set(0);

    // }

    }


