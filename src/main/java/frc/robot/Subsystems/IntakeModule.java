package frc.robot.Subsystems;

/*import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.WPIUtilJNI;
import frc.robot.Constants.DriveConstants;
import frc.utils.SwerveUtils;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.sensors.WPI_PigeonIMU;*/
//import edu.wpi.first.wpilibj.ADIS16470_IMU;
//import edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.BeltConstants;

public class IntakeModule {
   
private CANSparkMax conveyorBelt; 

public IntakeModule(int deviceId){

    conveyorBelt = new CANSparkMax(deviceId,MotorType.kBrushless);

}

    public void reject(){
        double beltRejectFactor=SmartDashboard.getNumber("Belt Reject Factor", BeltConstants.k_beltRejectSpeed);
        if(Math.abs(beltRejectFactor)>1){
            beltRejectFactor=0;
        }
        if(beltRejectFactor==BeltConstants.k_beltRejectSpeed){
        conveyorBelt.set(BeltConstants.k_beltRejectSpeed);
        }
        else{
        conveyorBelt.set(beltRejectFactor);
        }
}
    // public void pickUp() {

    //     conveyorBelt.set(1);

    // }

    public void HighShoot() {

        double beltHighShootFactor=SmartDashboard.getNumber("Belt High Shoot Factor", BeltConstants.k_beltHighShootSpeed);
        if(Math.abs(beltHighShootFactor)>1){
            beltHighShootFactor=0;
        }
        if(beltHighShootFactor==BeltConstants.k_beltHighShootSpeed){
        conveyorBelt.set(BeltConstants.k_beltHighShootSpeed);
        }
        else{
        conveyorBelt.set(beltHighShootFactor);
        }

    }
    public void LowShoot() {

        double beltLowShootFactor=SmartDashboard.getNumber("Belt Low Shoot Factor", BeltConstants.k_beltLowShootSpeed);
        if(Math.abs(beltLowShootFactor)>1){
            beltLowShootFactor=0;
        }
        if(beltLowShootFactor==BeltConstants.k_beltLowShootSpeed){
        conveyorBelt.set(BeltConstants.k_beltLowShootSpeed);
        }
        else{
        conveyorBelt.set(beltLowShootFactor);
        }

    }
    public void PickUp() {

        double beltPickupFactor=SmartDashboard.getNumber("Belt Pickup Factor", BeltConstants.k_beltPickupSpeed);
        if(Math.abs(beltPickupFactor)>1){
            beltPickupFactor=0;
        }
        if(beltPickupFactor==BeltConstants.k_beltPickupSpeed){
        conveyorBelt.set(BeltConstants.k_beltPickupSpeed);
        }
        else{
        conveyorBelt.set(beltPickupFactor);
        }

    }
    public void still(){

        conveyorBelt.set(0);

    }
}








    

