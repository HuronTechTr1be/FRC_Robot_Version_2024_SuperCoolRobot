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
import frc.robot.Subsystems.SweeperWheelsBasic;
public class SweeperWheelsSubsystem extends SubsystemBase{

    private SweeperWheelsBasic m_left = new SweeperWheelsBasic(31);
    private SweeperWheelsBasic m_right = new SweeperWheelsBasic(32);

    public void PickUp(){
        m_left.PickUp();
        m_right.PickUp();
    }

    public void Reject(){
        m_left.Reject();
        m_right.Reject();
    }

    public void Reject(double speed){
        m_left.Reject(speed);
        m_right.Reject(speed);
    }

    public void Still(){
        m_left.Still();
        m_right.Still();
    }
}
