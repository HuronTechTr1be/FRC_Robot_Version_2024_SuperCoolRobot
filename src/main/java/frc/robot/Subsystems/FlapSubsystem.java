package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class FlapSubsystem extends SubsystemBase {
    
    private CANSparkMax flap;

    public FlapSubsystem(int deviceId){

        flap = new CANSparkMax(deviceId,MotorType.kBrushless);

    }

    public void Up(){

        flap.set(1);

    }

    public void Down(){

        flap.set(-1);

    }

    public void Still(){

        flap.set(0);

    }

}
