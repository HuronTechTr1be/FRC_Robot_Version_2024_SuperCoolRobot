
package frc.robot.Subsystems;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition; 
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.WPIUtilJNI;
import com.revrobotics.CANSparkMax;
import com.ctre.phoenix.sensors.WPI_PigeonIMU;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import frc.robot.Constants.DriveConstants;
import frc.utils.SwerveUtils;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.PS4Controller;

public class ClawSubsystem extends SubsystemBase {
     
  CANSparkMax liftArmLeft = new CANSparkMax(21,MotorType.kBrushless);
  CANSparkMax liftArmRight = new CANSparkMax(22,MotorType.kBrushless);

    public void UppyDownyArmsLeft(double speed){

        liftArmLeft.set(speed);

    }

    public void UppyDownyArmsRight(double speed){

         liftArmRight.set(speed);

    }

}
