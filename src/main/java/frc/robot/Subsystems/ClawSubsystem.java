
package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class ClawSubsystem extends SubsystemBase {

  private CANSparkMax arm;

  public ClawSubsystem(int deviceId) {

    arm = new CANSparkMax(deviceId,MotorType.kBrushless);

  }

  public void UppyDownyArmsUp() {

    arm.set(1);

  }

  public void UppyDownyArmsDown() {

    arm.set(-1);

  }
  public void UppyDownyArmsStill() {

    arm.set(0);
  }
}
