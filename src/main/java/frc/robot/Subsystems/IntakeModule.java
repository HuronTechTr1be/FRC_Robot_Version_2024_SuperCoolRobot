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

public class IntakeModule {
   
private CANSparkMax conveyorBelt; 

public IntakeModule(int deviceId){

    conveyorBelt = new CANSparkMax(deviceId,MotorType.kBrushless);

}

    public void reject(){

        conveyorBelt.set(-1);

}
    public void pickUp() {

        conveyorBelt.set(1);

    }
    public void HighShoot() {

        conveyorBelt.set(1);

    }
    public void LowShoot() {

        conveyorBelt.set(1);

    }
    public void PickUp() {

        conveyorBelt.set(1);

    }
    public void still(){

        conveyorBelt.set(0);

    }
}








    

