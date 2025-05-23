package org.firstinspires.ftc.teamcode.Subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ArmSubsystem extends SubsystemBase {
    private final MotorEx arm;
    private final PIDFController armPID;
    private double armTargetPosition = 0.0;

    private static final double kP = 0.015;
    private static final double kI = 0;
    private static final double kD = 0.0005;
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

    public void autoArmMover(double autoTargetArmPosition) {
        setPosition(autoTargetArmPosition);

        final long STUCK_TIMEOUT_MS = 1000;
        final double MOVEMENT_THRESHOLD = 0.5;
        final double POSITION_TOLERANCE = 2;

        long stuckStartTime = 0;
        double lastPosition = getArmPosition();

        while (Math.abs(autoTargetArmPosition - getArmPosition()) > POSITION_TOLERANCE) {
            update();

            double currentPosition = getArmPosition();
            double power = getArmPower();

            boolean isMoving = Math.abs(currentPosition - lastPosition) > MOVEMENT_THRESHOLD;

            if (power != 0 && !isMoving) {
                if (stuckStartTime == 0) {
                    stuckStartTime = System.currentTimeMillis();
                } else if (System.currentTimeMillis() - stuckStartTime > STUCK_TIMEOUT_MS) {
                    break; // Exit loop if stuck for too long
                }
            } else {
                stuckStartTime = 0; // Reset timer if moving again
            }

            lastPosition = currentPosition;
        }

        arm.set(0); // Stop the arm
    }




    public void setArmPower(double power) {
        arm.set(power);
    }
}