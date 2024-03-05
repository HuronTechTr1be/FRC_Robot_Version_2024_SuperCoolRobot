package frc.robot.Subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.BeltConstants;

public class IntakeModule extends SubsystemBase {

    private CANSparkMax conveyorBelt;

    public IntakeModule(int deviceId) {

        conveyorBelt = new CANSparkMax(deviceId, MotorType.kBrushless);

    }

    public void Reject() {
        double beltRejectFactor = SmartDashboard.getNumber("Belt Reject Factor", BeltConstants.k_beltRejectSpeed);
        if (Math.abs(beltRejectFactor) > 1) {
            beltRejectFactor = 0;
        }
        if (beltRejectFactor == BeltConstants.k_beltRejectSpeed) {
            conveyorBelt.set(BeltConstants.k_beltRejectSpeed);
        } else {
            conveyorBelt.set(beltRejectFactor);
        }
    }

    public void Reject(double speed) {

        if (speed > 0) {
            speed *= -1;
        }
        conveyorBelt.set(speed);

    }

    public void HighShoot() {

        double beltHighShootFactor = SmartDashboard.getNumber("Belt High Shoot Factor",
                BeltConstants.k_beltHighShootSpeed);
        if (Math.abs(beltHighShootFactor) > 1) {
            beltHighShootFactor = 0;
        }
        if (beltHighShootFactor == BeltConstants.k_beltHighShootSpeed) {
            conveyorBelt.set(BeltConstants.k_beltHighShootSpeed);
        } else {
            conveyorBelt.set(beltHighShootFactor);
        }

    }

    public void LowShoot() {

        double beltLowShootFactor = SmartDashboard.getNumber("Belt Low Shoot Factor",
                BeltConstants.k_beltLowShootSpeed);
        if (Math.abs(beltLowShootFactor) > 1) {
            beltLowShootFactor = 0;
        }
        if (beltLowShootFactor == BeltConstants.k_beltLowShootSpeed) {
            conveyorBelt.set(BeltConstants.k_beltLowShootSpeed);
        } else {
            conveyorBelt.set(beltLowShootFactor);
        }

    }

    public void PickUp() {

        double beltPickupFactor = SmartDashboard.getNumber("Belt Pickup Factor", BeltConstants.k_beltPickupSpeed);
        if (Math.abs(beltPickupFactor) > 1) {
            beltPickupFactor = 0;
        }
        if (beltPickupFactor == BeltConstants.k_beltPickupSpeed) {
            conveyorBelt.set(BeltConstants.k_beltPickupSpeed);
        } else {
            conveyorBelt.set(beltPickupFactor);
        }

    }

    public void Still() {

        conveyorBelt.set(0);

    }
}
