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
    
private EgressBasic m_top = new EgressBasic(42);
private EgressBasic m_bottom = new EgressBasic(41);    

public void PickUp(){
  m_top.PickUp();
  m_bottom.PickUp();
}

public void Reject(){
  m_top.Reject();
  m_bottom.Reject();
}

public void HighShoot(){
  m_top.HighShoot();
  m_bottom.HighShoot();
}

public void LowShoot(){
  m_top.LowShoot();
  m_bottom.LowShoot();
}

public void adjustedLowShoot(double shootPower){
  m_top.adjustedLowShoot(shootPower);
  m_bottom.adjustedLowShoot(shootPower);
}

public void Still(){
  m_bottom.Still();
  m_top.Still();
}

}
