// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

// import java.util.List;
import java.util.Optional;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.Subsystems.LEDSubsystem;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoSink;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.OIConstants;
import frc.robot.BlueAutons.BLUESpeakerLeftNote;
import frc.robot.BlueAutons.BLUESpeakerRightNote;
import frc.robot.BlueAutons.BOTHLeftWaitRetreat;
import frc.robot.BlueAutons.BOTHRightWaitRetreat;
import frc.robot.BlueAutons.BOTHSpeakerMiddleNote;
import frc.robot.Commands.FlapDownCommand;
import frc.robot.Commands.FlapUpCommand;
import frc.robot.Commands.HighShootCommand;
import frc.robot.Commands.LowShootCommand;
import frc.robot.Commands.MotorsStillCommand;
import frc.robot.Commands.PickUpCommand;
import frc.robot.Commands.RejectCommand;
import frc.robot.Commands.SlowRejectCommand;
import frc.robot.RedAutons.REDSpeakerLeftNote;
import frc.robot.RedAutons.REDSpeakerRightNote;
import frc.robot.Subsystems.DriveSubsystem;
import frc.robot.Subsystems.EgressSubsystem;
import frc.robot.Subsystems.FlapSubsystem;
import frc.robot.Subsystems.IntakeModule;
import frc.robot.Subsystems.SweeperWheelsSubsystem;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private EgressSubsystem m_Shoot = new EgressSubsystem();
  private IntakeModule m_conveyorBelt = new IntakeModule(33);
  private SweeperWheelsSubsystem m_SweeperWheels = new SweeperWheelsSubsystem();
  private final FlapSubsystem m_robotFlap = new FlapSubsystem(51);

  DigitalInput autonSwitch1 = new DigitalInput(9);
  DigitalInput autonSwitch2 = new DigitalInput(8);
  DigitalInput autonSwitch3 = new DigitalInput(7);
  DigitalInput autonSwitch4 = new DigitalInput(6);
  DigitalInput autonSwitch5 = new DigitalInput(5);
  DigitalInput autonSwitch6 = new DigitalInput(4);

  DigitalInput photoElectricSensor = new DigitalInput(0);

  // The driver's controller
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  PS4Controller m_shooterController = new PS4Controller(1);

  JoystickButton CrossButton = new JoystickButton(m_shooterController, PS4Controller.Button.kCross.value);
  JoystickButton CircleButton = new JoystickButton(m_shooterController, PS4Controller.Button.kCircle.value);
  JoystickButton SquareButton = new JoystickButton(m_shooterController, PS4Controller.Button.kSquare.value);
  JoystickButton TriangleButton = new JoystickButton(m_shooterController, PS4Controller.Button.kTriangle.value);
  JoystickButton ShooterLeftBumper = new JoystickButton(m_shooterController, PS4Controller.Button.kL2.value);
  JoystickButton ShooterRightBumper = new JoystickButton(m_shooterController, PS4Controller.Button.kR2.value);
  JoystickButton ShooterLeftTrigger = new JoystickButton(m_shooterController, PS4Controller.Button.kL1.value);
  JoystickButton ShooterRightTrigger = new JoystickButton(m_shooterController, PS4Controller.Button.kR1.value);

  HighShootCommand HighShoot = new HighShootCommand(m_Shoot, m_conveyorBelt);
  LowShootCommand LowShoot = new LowShootCommand(m_Shoot, m_conveyorBelt, 0.3);
  SlowRejectCommand SlowReject = new SlowRejectCommand(m_Shoot, m_conveyorBelt, m_SweeperWheels);
  RejectCommand Reject = new RejectCommand(m_Shoot, m_conveyorBelt, m_SweeperWheels);
  PickUpCommand PickUp = new PickUpCommand(m_Shoot, m_conveyorBelt, m_SweeperWheels);
  FlapUpCommand FlapUp = new FlapUpCommand(m_robotFlap);
  FlapDownCommand FlapDown = new FlapDownCommand(m_robotFlap);
  MotorsStillCommand MotorsStill = new MotorsStillCommand(m_Shoot, m_conveyorBelt, m_SweeperWheels, m_robotFlap);

  BOTHSpeakerMiddleNote m_BOTHSpeakerMiddleNote = new BOTHSpeakerMiddleNote(m_robotDrive, m_Shoot, m_conveyorBelt,
      m_SweeperWheels, m_robotFlap);

  // BLUEAmpRightNote m_BLUEAmpRightNote = new BLUEAmpRightNote(m_robotDrive,
  // m_Shoot, m_conveyorBelt, m_SweeperWheels, m_robotFlap);
  // BLUESpeakerLeftShootRetreat m_BLUESpeakerLeftShootRetreat = new
  // BLUESpeakerLeftShootRetreat(m_robotDrive, m_Shoot, m_conveyorBelt,
  // m_SweeperWheels, m_robotFlap);
  BLUESpeakerLeftNote m_BLUESpeakerLeftNote = new BLUESpeakerLeftNote(m_robotDrive, m_Shoot, m_conveyorBelt,
      m_SweeperWheels, m_robotFlap);
  BLUESpeakerRightNote m_BLUESpeakerRightNote = new BLUESpeakerRightNote(m_robotDrive, m_Shoot, m_conveyorBelt,
      m_SweeperWheels, m_robotFlap);
  // BLUESpeakerMiddleRightNotes
  // BLUESpeakerAllNotes
  // BLUESpeakerMiddleRightNotes
  // BLUEAmpMiddleRightNotes
  BOTHLeftWaitRetreat m_BothLeftWaitRetreat = new BOTHLeftWaitRetreat(m_robotDrive, m_Shoot, m_conveyorBelt,
      m_SweeperWheels, m_robotFlap);
  BOTHRightWaitRetreat m_BothRightWaitRetreat = new BOTHRightWaitRetreat(m_robotDrive, m_Shoot, m_conveyorBelt,
      m_SweeperWheels, m_robotFlap);

  // REDAmpRightNote m_REDAmpRightNote = new REDAmpRightNote(m_robotDrive,
  // m_Shoot, m_conveyorBelt, m_SweeperWheels, m_robotFlap);
  // REDSpeakerLeftShootRetreat m_REDSpeakerLeftShootRetreat = new
  // REDSpeakerLeftShootRetreat(m_robotDrive, m_Shoot, m_conveyorBelt,
  // m_SweeperWheels, m_robotFlap);
  REDSpeakerLeftNote m_REDSpeakerLeftNote = new REDSpeakerLeftNote(m_robotDrive, m_Shoot, m_conveyorBelt,
      m_SweeperWheels, m_robotFlap);
  REDSpeakerRightNote m_REDSpeakerRightNote = new REDSpeakerRightNote(m_robotDrive, m_Shoot, m_conveyorBelt,
      m_SweeperWheels, m_robotFlap);
  // REDSpeakerMiddleRightNotes
  // REDSpeakerAllNotes
  // REDSpeakerMiddleRightNotes
  // REDAmpMiddleRightNotes

  SendableChooser<Command> m_chooser = new SendableChooser<>();

  public void resetReverseDrive() {
    m_robotDrive.resetReverseDrive();
  }

  public void DriveRampRate0(){
    m_robotDrive.setRampRate0();
  }

  public void DriveRampRate3(){
    m_robotDrive.setRampRate3();
  }

  public void periodic() {
    SmartDashboard.putBoolean("photoElectricSensor", photoElectricSensor.get());
    if (!(CrossButton.getAsBoolean() || SquareButton.getAsBoolean() || CircleButton.getAsBoolean()
        || TriangleButton.getAsBoolean() || ShooterLeftBumper.getAsBoolean() || ShooterRightBumper.getAsBoolean()
        || ShooterLeftTrigger.getAsBoolean() || ShooterRightTrigger.getAsBoolean())) {
      m_Shoot.Still();
      m_SweeperWheels.Still();
      m_conveyorBelt.Still();
    }
    SmartDashboard.putBoolean("Flap Limit", m_robotFlap.isRaised());

  }

  public boolean LEDBase = true;

  public void LEDFunctions(LEDSubsystem ledSubsystem, Optional<Alliance> ally) {
    if (LEDBase) {
      if (TriangleButton.getAsBoolean()) {
        if (!(photoElectricSensor.get())) {
          ledSubsystem.setAll(Color.kGreen);
          LEDBase = false;
          m_driverController.setRumble(GenericHID.RumbleType.kBothRumble, 1);
          m_shooterController.setRumble(GenericHID.RumbleType.kBothRumble, 1);

        }
      }
    }
    if (!LEDBase) {
      if ((!(CrossButton.getAsBoolean() || SquareButton.getAsBoolean() || CircleButton.getAsBoolean()
          || TriangleButton.getAsBoolean() || ShooterLeftBumper.getAsBoolean() || ShooterRightBumper.getAsBoolean()
          || ShooterLeftTrigger.getAsBoolean() || ShooterRightTrigger.getAsBoolean()))) {
        if (ally.get() == Alliance.Blue) {
          ledSubsystem.setAll(Color.kBlue);
        } else if (ally.get() == Alliance.Red) {
          ledSubsystem.setAll(Color.kRed);
        }
        LEDBase = true;
        m_driverController.setRumble(GenericHID.RumbleType.kBothRumble, 0);
        m_shooterController.setRumble(GenericHID.RumbleType.kBothRumble, 0);

      }
    }
  }

  boolean movingUp = false;
  boolean movingDown = false;

  public void FlapRun() {

    if (ShooterRightBumper.getAsBoolean()) {
      m_robotFlap.FlapUp(0.5);
      movingUp = true;
      movingDown = false;
    } else if (ShooterRightTrigger.getAsBoolean()) {
      m_robotFlap.FlapDown();
      movingDown = true;
      movingUp = false;
    }
    if (movingUp) {
      if (m_robotFlap.isRaised()) {
        movingUp = false;
        m_robotFlap.FlapStill();
      }
    }
    if (movingDown) {
      if (m_robotFlap.isLowered()) {
        movingDown = false;
        m_robotFlap.FlapStill();
      }
    }

  }

  public void initReset() {
    m_robotDrive.resetFrontRightEncoder();
  }

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    // Configure the button bindings
    configureButtonBindings();

    m_chooser.setDefaultOption("BOTHSpeakerMiddleNote", m_BOTHSpeakerMiddleNote);
    m_chooser.addOption("Blue Left", m_BLUESpeakerLeftNote);
    m_chooser.addOption("Blue Right", m_BLUESpeakerRightNote);
    m_chooser.addOption("Red Left", m_REDSpeakerLeftNote);
    m_chooser.addOption("Red Right", m_REDSpeakerRightNote);

    SmartDashboard.putData(m_chooser);

    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
                false, true),
            m_robotDrive));
  }

  public void resetRobot() {

    m_robotDrive.resetClaws();
    m_robotFlap.flapSetZero();

  }

  // int x = 0;
  // int y = 0;

  // public void cameraSwitch(UsbCamera camera1, UsbCamera camera2, VideoSink server) {
  //   if (ShooterLeftTrigger.getAsBoolean()) {
  //     if (x == 0) {
  //       System.out.println("Setting camera 2");
  //       server.setSource(camera2);
  //       x = 1;
  //     }
  //     y = 0;

  //   } else if (!ShooterLeftTrigger.getAsBoolean()) {
  //     if (y == 0) {
  //       System.out.println("Setting camera 1");
  //       server.setSource(camera1);
  //       y = 1;
  //     }
  //     x = 0;
  //   }
  // }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    new JoystickButton(m_driverController, XboxController.Button.kX.value)
        .whileTrue(new RunCommand(
            () -> m_robotDrive.setX(),
            m_robotDrive));
    CircleButton.whileTrue(HighShoot);
    SquareButton.whileTrue(LowShoot);
    CrossButton.whileTrue(SlowReject);
    TriangleButton.whileTrue(PickUp);
    ShooterLeftBumper.whileTrue(Reject);

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {

    return m_chooser.getSelected();

    // if (!autonSwitch1.get()) {
    // return m_BOTHSpeakerMiddleNote; // 1
    // //return m_BothLeftWaitRetreat;
    // //return m_BothRightWaitRetreat;
    // }
    // if (!autonSwitch2.get()) {
    // return m_BLUESpeakerLeftNote; // 2
    // }
    // if (!autonSwitch3.get()) {
    // return m_BLUESpeakerRightNote; // 3
    // }
    // if (!autonSwitch4.get()) {
    // return m_REDSpeakerLeftNote; // 4
    // }
    // if (!autonSwitch5.get()) {
    // return m_REDSpeakerRightNote; // 5
    // }

  }

}