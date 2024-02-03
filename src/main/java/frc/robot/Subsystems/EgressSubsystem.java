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
import com.ctre.phoenix.sensors.WPI_PigeonIMU;
import frc.robot.Constants.DriveConstants;
import frc.utils.SwerveUtils;
//import edu.wpi.first.wpilibj.ADIS16470_IMU;
//import edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis;*/

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import frc.robot.Constants.EgressConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

   
public class EgressSubsystem extends SubsystemBase {
    
private CANSparkMax shooterWheel;    

    
public EgressSubsystem(int deviceId) {

    shooterWheel = new CANSparkMax(deviceId,MotorType.kBrushless);

  }

  public void HighShoot() {

    if(shooterWheel.getDeviceId()==41){
    shooterWheel.set(EgressConstants.id41HighShootFactor);
    }
    else if(shooterWheel.getDeviceId()==42){
    shooterWheel.set(EgressConstants.id42HighShootFactor);
    }

}

public void LowShoot() {

    if(shooterWheel.getDeviceId()==41){
    shooterWheel.set(EgressConstants.id41LowShootFactor);
    }
    else if(shooterWheel.getDeviceId()==42){
    shooterWheel.set(EgressConstants.id42LowShootFactor);
    }

   }

  public void Reject() {

    if(shooterWheel.getDeviceId()==41){
    shooterWheel.set(EgressConstants.id41RejectFactor);
    }
    else if(shooterWheel.getDeviceId()==42){
    shooterWheel.set(EgressConstants.id42RejectFactor);
    }

  }
  public void PickUp() {

    if(shooterWheel.getDeviceId()==41){
    shooterWheel.set(EgressConstants.id41PickUpFactor);
    }
    else if(shooterWheel.getDeviceId()==42){
    shooterWheel.set(EgressConstants.id42PickUpFactor);
    }

  }
  public void Still() {

    shooterWheel.set(0);

  }

}
