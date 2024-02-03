
package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;


public class ClawSubsystem extends SubsystemBase {

  private CANSparkMax arm;

  //public final AbsoluteEncoder m_armEndoEncoder;
  //m_armEncoder = arm.getEncoder();

  public ClawSubsystem(int deviceId) {

    arm = new CANSparkMax(deviceId,MotorType.kBrushless);

  }

  public double ArmEncoder(){

    return arm.getEncoder().getPosition();
    
  }
  
  public void UppyDownyArmsUp() {

    arm.set(1);

  }

  public void UppyDownyArmsUp(double speed) {

    arm.set(speed);

  }

  public void UppyDownyArmsDown() {

    arm.set(-1);

  }
  
  public void UppyDownyArmsStill() {

    arm.set(0);
  }
}
