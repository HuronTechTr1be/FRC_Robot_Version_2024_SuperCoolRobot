package frc.robot.Subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import frc.robot.Constants.DriveConstants;
//import frc.robot.Constants.EgressConstants;
//import frc.utils.SwerveUtils;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.BeltConstants;
import frc.robot.Constants.SweeperWheelConstants;
public class SweeperWheelsBasic{

    private CANSparkMax sweeperWheel;
 
    public SweeperWheelsBasic(int deviceId){

        sweeperWheel = new CANSparkMax(deviceId,MotorType.kBrushless);

    }

    public void PickUp() {

        if(sweeperWheel.getDeviceId()==31){
            double sweeperWheel31PickupFactor=SmartDashboard.getNumber("Sweeper Wheel 31 Pickup Factor", SweeperWheelConstants.id31PickUpFactor);
        if(Math.abs(sweeperWheel31PickupFactor)>1){
            sweeperWheel31PickupFactor=0;
        }
        if(sweeperWheel31PickupFactor==SweeperWheelConstants.id31PickUpFactor){
        sweeperWheel.set(SweeperWheelConstants.id31PickUpFactor);
        }
        else{
        sweeperWheel.set(sweeperWheel31PickupFactor);
        }
            
    }
            else if(sweeperWheel.getDeviceId()==32){
            double sweeperWheel32PickupFactor=SmartDashboard.getNumber("Sweeper Wheel 32 Pickup Factor", SweeperWheelConstants.id32PickUpFactor);
        if(Math.abs(sweeperWheel32PickupFactor)>1){
            sweeperWheel32PickupFactor=0;
        }
        if(sweeperWheel32PickupFactor==SweeperWheelConstants.id32PickUpFactor){
        sweeperWheel.set(SweeperWheelConstants.id32PickUpFactor);
        }
        else{
        sweeperWheel.set(sweeperWheel32PickupFactor);
        }
         }
    }

    public void Reject() {

        if(sweeperWheel.getDeviceId()==31){
            double sweeperWheel31RejectFactor=SmartDashboard.getNumber("Sweeper Wheel 31 Reject Factor", SweeperWheelConstants.id31RejectFactor);
        if(Math.abs(sweeperWheel31RejectFactor)>1){
            sweeperWheel31RejectFactor=0;
        }
        if(sweeperWheel31RejectFactor==SweeperWheelConstants.id31RejectFactor){
        sweeperWheel.set(SweeperWheelConstants.id31RejectFactor);
        }
        else{
        sweeperWheel.set(sweeperWheel31RejectFactor);
        }
            
    }
            else if(sweeperWheel.getDeviceId()==32){
            double sweeperWheel32RejectFactor=SmartDashboard.getNumber("Sweeper Wheel 32 Reject Factor", SweeperWheelConstants.id32RejectFactor);
        if(Math.abs(sweeperWheel32RejectFactor)>1){
            sweeperWheel32RejectFactor=0;
        }
        if(sweeperWheel32RejectFactor==SweeperWheelConstants.id32RejectFactor){
        sweeperWheel.set(SweeperWheelConstants.id32RejectFactor);
        }
        else{
        sweeperWheel.set(sweeperWheel32RejectFactor);
        }
         }

    }

    public void Still() {

        sweeperWheel.set(0);

        }

    }
