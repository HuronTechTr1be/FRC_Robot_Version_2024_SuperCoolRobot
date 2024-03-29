package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.DriveSubsystem;

public class DriveCommandDistance extends Command {

  private final DriveSubsystem m_Drive;
  private final double xSpeed;
  private final double ySpeed;
  private final double rot;
  private final double distance;

  public DriveCommandDistance(DriveSubsystem subsystem, double XSpeed, double YSpeed, double Rot,
      double driveDistance) {
    m_Drive = subsystem;
    xSpeed = XSpeed;
    ySpeed = YSpeed;
    rot = Rot;
    distance = driveDistance;
    addRequirements(m_Drive);
  }

  @Override
  public void initialize() {
    m_Drive.drive(xSpeed, ySpeed, rot, false, true);
    // while (Math.abs(m_Drive.getFrontRightEncoder()) < Math.abs(distance)) {

    // }
    // m_Drive.drive(0, 0, 0, false, true);
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
