package org.firstinspires.ftc.teamcode.Subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class SubArmSubsystem extends SubsystemBase {
    private final MotorEx subArm;
    private final PIDFController subArmPID;
    private double armTargetPosition = 0.0;

    private static final double kP = 0.01;
    private static final double kI = 0;
    private static final double kD = 0;
    private static final double kF = 0;
    private static final double kG = 0;


    public SubArmSubsystem(HardwareMap hardwareMap) {
        subArm = new MotorEx(hardwareMap, "arm");
        subArm.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        subArmPID = new PIDFController(kP, kI, kD, kF);
    }

    public void update() {
        double currentPosition = subArm.getCurrentPosition();

        double gravityCompensation = kG * Math.cos(Math.toRadians(currentPosition));

        double armPower = subArmPID.calculate(currentPosition, armTargetPosition) + gravityCompensation;

        subArm.set(armPower);
    }

    public void setPosition(double position) {
        armTargetPosition = position;
    }

    public double getArmTargetPosition() {
        return armTargetPosition;
    }

    public double getArmPower() {
        return subArmPID.calculate(subArm.getCurrentPosition(), armTargetPosition);
    }

    public void resetZero() {
        subArm.stopAndResetEncoder();
    }

    public double getArmPosition() {
        return subArm.getCurrentPosition();
    }

    public double getArmError() {
        // this is for auto only because we cant use same subsystem at the same time
        return Math.abs(getArmTargetPosition()) - Math.abs(getArmPosition());
    }

    public void autoArmMover(double autoTargetArmPosition){
        setPosition(autoTargetArmPosition);
        while (Math.abs(autoTargetArmPosition - getArmPosition()) > 5){
            update();
        }
        subArm.set(0);
    }

    public void setArmPower(double power) {
        subArm.set(power);
    }
}