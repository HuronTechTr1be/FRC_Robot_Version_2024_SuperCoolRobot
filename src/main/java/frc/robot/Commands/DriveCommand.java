package frc.robot.Commands;
import frc.robot.Subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

  public class DriveCommand extends Command {
  // The subsystem the command runs on
    private final DriveSubsystem m_Drive;
    private final double xSpeed;
    private final double ySpeed;
    private final double rot;


  public DriveCommand(DriveSubsystem subsystem, double XSpeed, double YSpeed, double Rot) {
    m_Drive = subsystem;
    xSpeed = XSpeed;
    ySpeed = YSpeed;
    rot = Rot;
    addRequirements(m_Drive);
  }

  @Override
  public void initialize() {
    m_Drive.drive(xSpeed, ySpeed, rot, false, true);
    
}

  @Override
  public boolean isFinished() {
    return true;
  }
}
