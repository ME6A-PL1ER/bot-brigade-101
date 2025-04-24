package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class AmpMonitor {
    private final DcMotorEx arm;

    public AmpMonitor(DcMotorEx arm) {
        this.arm = arm;
    }

    public double getCurrentAmperage() {
        return arm.getCurrent(CurrentUnit.AMPS);
    }

    public void protectMotor(double threshold) {
        double currentAmperage = getCurrentAmperage();
        if (currentAmperage > threshold) {
            arm.setPower(0);
        }
    }


    public boolean ampTooHigh(double warningThreshold) {
        double currentAmperage = getCurrentAmperage();
        if (currentAmperage > warningThreshold) {
            return true;
        }
        return false;
    }
}