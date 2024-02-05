package frc.robot.Subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
//import frc.robot.Constants.DriveConstants;
//import frc.robot.Constants.EgressConstants;
//import frc.utils.SwerveUtils;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SweeperWheelConstants;
public class SweeperWheelsSubsystem extends SubsystemBase{

    private CANSparkMax sweeperWheel;
 
    public SweeperWheelsSubsystem(int deviceId){

        sweeperWheel = new CANSparkMax(deviceId,MotorType.kBrushless);

    }

    public void PickUp() {

        if(sweeperWheel.getDeviceId()==31){
            sweeperWheel.set(SweeperWheelConstants.id31PickUpFactor);
            }
            else if(sweeperWheel.getDeviceId()==32){
            sweeperWheel.set(SweeperWheelConstants.id32PickUpFactor);
         }
    }

    public void Reject() {

        if(sweeperWheel.getDeviceId()==31){
            sweeperWheel.set(SweeperWheelConstants.id31RejectFactor);
            }
            else if(sweeperWheel.getDeviceId()==32){
            sweeperWheel.set(SweeperWheelConstants.id32RejectFactor);
        }

    }

    public void Still() {

        sweeperWheel.set(0);

        }

    }
