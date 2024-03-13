package frc.robot.Subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import frc.robot.Constants.EgressConstants;

public class EgressBasic {

  private CANSparkMax shooterWheel;

  public EgressBasic(int deviceId) {

    shooterWheel = new CANSparkMax(deviceId, MotorType.kBrushless);

  }

  public void adjustedHighShoot(double speed) {
    shooterWheel.set(speed);
  }

  public void HighShoot() {

    if (shooterWheel.getDeviceId() == 41) {
      shooterWheel.set(EgressConstants.id41HighShootFactor);
    } else if (shooterWheel.getDeviceId() == 42) {
      shooterWheel.set(EgressConstants.id42HighShootFactor);
    } else if (shooterWheel.getDeviceId() == 43) {
      shooterWheel.set(EgressConstants.id43HighShootFactor);
    } else if (shooterWheel.getDeviceId() == 44) {
      shooterWheel.set(EgressConstants.id44HighShootFactor);
    }

  }

  public void adjustedLowShoot(double speed) {
    shooterWheel.set(speed);
  }

  public void LowShoot() {

    if (shooterWheel.getDeviceId() == 41) {
      shooterWheel.set(EgressConstants.id41LowShootFactor);
    } else if (shooterWheel.getDeviceId() == 42) {
      shooterWheel.set(EgressConstants.id42LowShootFactor);
    } else if (shooterWheel.getDeviceId() == 43) {
      shooterWheel.set(EgressConstants.id43LowShootFactor);
    } else if (shooterWheel.getDeviceId() == 44) {
      shooterWheel.set(EgressConstants.id44LowShootFactor);
    }

  }

  public void Reject() {

    if (shooterWheel.getDeviceId() == 41) {
      shooterWheel.set(EgressConstants.id41RejectFactor);
    } else if (shooterWheel.getDeviceId() == 42) {
      shooterWheel.set(EgressConstants.id42RejectFactor);
    } else if (shooterWheel.getDeviceId() == 43) {
      shooterWheel.set(EgressConstants.id43RejectFactor);
    } else if (shooterWheel.getDeviceId() == 44) {
      shooterWheel.set(EgressConstants.id44RejectFactor);
    }

  }

  public void Reject(double speed) {
    if (speed > 0) {
      speed *= -1;
    }
    if (shooterWheel.getDeviceId() == 41) {
      shooterWheel.set(speed);
    } else if (shooterWheel.getDeviceId() == 43) {
      shooterWheel.set(speed);
    }
  }

  public void PickUp() {

    if (shooterWheel.getDeviceId() == 41) {
      shooterWheel.set(EgressConstants.id41PickUpFactor);
    } else if (shooterWheel.getDeviceId() == 42) {
      shooterWheel.set(EgressConstants.id42PickUpFactor);
    } else if (shooterWheel.getDeviceId() == 43) {
      shooterWheel.set(EgressConstants.id43PickUpFactor);
    } else if (shooterWheel.getDeviceId() == 44) {
      shooterWheel.set(EgressConstants.id44PickUpFactor);
    }

  }

  public void Still() {

    shooterWheel.set(0);

  }

}
