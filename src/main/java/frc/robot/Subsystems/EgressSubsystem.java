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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

   
public class EgressSubsystem extends SubsystemBase {
    
private CANSparkMax shooterWheel;    

    
public EgressSubsystem(int deviceId) {

    shooterWheel = new CANSparkMax(deviceId,MotorType.kBrushless);

  }
  public void adjustedHighShoot(double speed) {
    shooterWheel.set(speed);
  }

  public void HighShoot() {
    
    if(shooterWheel.getDeviceId()==41){
      shooterWheel.set(EgressConstants.id41HighShootFactor);
    }
    else if(shooterWheel.getDeviceId()==42){
      shooterWheel.set(EgressConstants.id42HighShootFactor);
    }
    
    // double bottomHighShootFactor = SmartDashboard.getNumber("High Shoot Factor:", EgressConstants.id41HighShootFactor);
    // double topHighShootFactor = SmartDashboard.getNumber("High Shoot Factor:", EgressConstants.id42HighShootFactor);
    // if(shooterWheel.getDeviceId()==41){
    // if(bottomHighShootFactor == EgressConstants.id41HighShootFactor){
    //   shooterWheel.set(EgressConstants.id41HighShootFactor);
    // }
    // else{
    //   shooterWheel.set(bottomHighShootFactor);
    // }
    //   }

    // else if(shooterWheel.getDeviceId()==42){
    //   if(topHighShootFactor == EgressConstants.id42HighShootFactor){
    //   shooterWheel.set(EgressConstants.id42HighShootFactor);
    // }
    // else{
    //   shooterWheel.set(topHighShootFactor);
    // }
    // }

}

public void adjustedLowShoot(double speed) {
  shooterWheel.set(speed);
}

public void LowShoot() {

    // double bottomLowShootFactor = SmartDashboard.getNumber("Bottom Low Shoot Factor:", EgressConstants.id41LowShootFactor);
    // double topLowShootFactor = SmartDashboard.getNumber("High Low Shoot Factor:", EgressConstants.id42LowShootFactor);

    if(shooterWheel.getDeviceId()==41){
      shooterWheel.set(EgressConstants.id41LowShootFactor);
    }
    else if(shooterWheel.getDeviceId()==42){
      shooterWheel.set(EgressConstants.id42LowShootFactor);
    }


    // if(shooterWheel.getDeviceId()==41){
    // if(bottomLowShootFactor == EgressConstants.id41LowShootFactor){
    //   shooterWheel.set(EgressConstants.id41LowShootFactor);
    // }
    // else{
    //   shooterWheel.set(bottomLowShootFactor);
    // }
    //   }

    // else if(shooterWheel.getDeviceId()==42){
    //   if(topLowShootFactor == EgressConstants.id42LowShootFactor){
    //   shooterWheel.set(EgressConstants.id42LowShootFactor);
    // }
    // else{
    //   shooterWheel.set(topLowShootFactor);
    // }
    // }

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
