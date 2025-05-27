package org.firstinspires.ftc.teamcode.Main;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Subsystems.ClawSubsystem;

@TeleOp(name = "turning pid tune", group = "Drive")
public class PidTune extends LinearOpMode {
    private double kP = 0.11;
    private double kI = 0;
    private double kD = 1;

    boolean prevDpadUp = false;
    boolean prevDpadDown = false;
    boolean prevDpadRight = false;
    boolean prevDpadLeft = false;
    boolean prevY = false;
    boolean prevA = false;

    private IMU imu;

    private boolean timerStarted;
    ElapsedTime stabilityTimer = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor leftDrive = hardwareMap.dcMotor.get("leftDrive");
        DcMotor rightDrive = hardwareMap.dcMotor.get("rightDrive");

        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        waitForStart();

        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        imu.initialize(parameters);
        imu.resetYaw();

        while (opModeIsActive()) {

            if (gamepad1.dpad_up && !prevDpadUp) {
                kP += 0.005;
            }
            prevDpadUp = gamepad1.dpad_up;

            if (gamepad1.dpad_down && !prevDpadDown) {
                kP -= 0.005;
            }
            prevDpadDown = gamepad1.dpad_down;

            if (gamepad1.dpad_right && !prevDpadRight) {
                kI += 0.005;
            }
            prevDpadRight = gamepad1.dpad_right;

            if (gamepad1.dpad_left && !prevDpadLeft) {
                kI -= 0.005;
            }
            prevDpadLeft = gamepad1.dpad_left;

            if (gamepad1.y && !prevY) {
                kD += 0.005;
            }
            prevY = gamepad1.y;

            if (gamepad1.a && !prevA) {
                kD -= 0.005;
            }
            prevA = gamepad1.a;

            if (gamepad1.left_bumper) {rotateToAngle(leftDrive, rightDrive, 90);}
            if (gamepad1.right_bumper) {rotateToAngle(leftDrive, rightDrive, -90);}

            telemetry.addData("P value:", kP);
            telemetry.addData("I value:", kI);
            telemetry.addData("D value:", kD);
            telemetry.update();
        }
    }
    public void rotateToAngle(DcMotor leftDrive, DcMotor rightDrive, double targetAngle) {

        double error;
        double lastError = 0;
        double integral = 0;
        double derivative;
        double power;

        while (opModeIsActive()) {
            double currentAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            error = targetAngle - currentAngle;

            if (error > 180) {
                error -= 360;
            } else if (error < -180) {
                error += 360;
            }

            integral += error;
            derivative = error - lastError;

            power = (kP * error) + (kI * integral) + (kD * derivative);

            leftDrive.setPower(-power);
            rightDrive.setPower(-power);

            lastError = error;

            telemetry.addData("current angle", currentAngle);
            telemetry.addData("target angle", targetAngle);
            telemetry.addData("error", error);

            if (Math.abs(error) < 3) {
                if (!timerStarted) {
                    stabilityTimer.reset();
                    timerStarted = true;
                }

                if (stabilityTimer.milliseconds() >= 200) {
                    leftDrive.setPower(0);
                    rightDrive.setPower(0);
                    break;
                }
            } else {
                timerStarted = false;
            }
        }
    }
}