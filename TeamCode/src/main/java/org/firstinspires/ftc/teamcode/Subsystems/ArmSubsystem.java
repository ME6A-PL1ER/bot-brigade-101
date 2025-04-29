package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ArmSubsystem extends SubsystemBase {
    private final MotorEx arm;
    private final PIDFController armPID;
    private double armTargetPosition = 0.0;

    private static final double kP = 0.008;
    private static final double kI = 0;
    private static final double kD = 0;
    private static final double kF = 0;
    private static final double kG = 0;


    public ArmSubsystem(HardwareMap hardwareMap) {
        arm = new MotorEx(hardwareMap, "arm");
        arm.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        armPID = new PIDFController(kP, kI, kD, kF);
    }

    public void update() {
        double currentPosition = arm.getCurrentPosition();

        double gravityCompensation = kG * Math.cos(Math.toRadians(currentPosition));

        double armPower = armPID.calculate(currentPosition, armTargetPosition) + gravityCompensation;

        arm.set(armPower);
    }

    public void setPosition(double position) {
        armTargetPosition = position;
    }

    public double getArmTargetPosition() {
        return armTargetPosition;
    }

    public double getArmPower() {
        return armPID.calculate(arm.getCurrentPosition(), armTargetPosition);
    }

    public void resetZero() {
        arm.stopAndResetEncoder();
    }

    public double getArmPosition() {
        return arm.getCurrentPosition();
    }

    public double getArmError() {
        // this is for auto only because we cant use same subsystem at the same time
        return Math.abs(getArmTargetPosition()) - Math.abs(getArmPosition());
    }

    public void autoArmMover(double autoTargetArmPosition){
        setPosition(autoTargetArmPosition);
        while (Math.abs(autoTargetArmPosition - getArmPosition()) > 25){
            update();
        }
        arm.set(0);
    }

    public void setArmPower(double power) {
        arm.set(power);
    }
}