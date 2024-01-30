
package frc.robot.Subsystems;

/*import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition; 
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.DriveConstants;
import frc.utils.SwerveUtils;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj2.command.Command;
import com.ctre.phoenix.sensors.WPI_PigeonIMU;
import edu.wpi.first.util.WPIUtilJNI;*/

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class ClawSubsystem extends SubsystemBase {

  private CANSparkMax arm; // = new CANSparkMax(21,MotorType.kBrushless);
  //CANSparkMax liftArmRight = new CANSparkMax(22,MotorType.kBrushless);

  public ClawSubsystem(int deviceId) {

    arm = new CANSparkMax(deviceId,MotorType.kBrushless);

  }

  public void UppyDownyArmsUp() {

    arm.set(20);

  }

  public void UppyDownyArmsDown() {

    arm.set(-20);

  }
  public void UppyDownyArmsStill() {

    arm.set(0);
  }
}
