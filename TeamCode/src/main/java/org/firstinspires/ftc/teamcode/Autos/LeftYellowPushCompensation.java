package org.firstinspires.ftc.teamcode.Autos;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


@Autonomous(name = "Left push with voltage compensation", group = "autos")
public class LeftYellowPushCompensation extends LinearOpMode {
    private DcMotor leftDrive;
    private DcMotor rightDrive;
    private VoltageSensor batteryVoltageSensor;
    private IMU imu;
    private double fieldOffset = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor leftDrive = hardwareMap.dcMotor.get("leftDrive");
        DcMotor rightDrive = hardwareMap.dcMotor.get("rightDrive");
        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT));
        imu.initialize(parameters);

        batteryVoltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");

        telemetry.addData("Status", "Initialized. Waiting for start...");
        telemetry.update();
        waitForStart();
        if (isStopRequested()) return;

        moveDrivetrain(leftDrive, rightDrive, 0.5, 2000);
        rotateToAngle(leftDrive,rightDrive, -45);
        moveDrivetrain(leftDrive, rightDrive, 0.5, 500);
        rotateToAngle(leftDrive, rightDrive, -160);
        moveDrivetrain(leftDrive, rightDrive, 0.5, 2000);
        moveDrivetrain(leftDrive, rightDrive, -0.5, 2000);
        rotateToAngle(leftDrive, rightDrive, -90);
        moveDrivetrain(leftDrive, rightDrive, 0.5, 500);
        rotateToAngle(leftDrive, rightDrive, 150);


    }
    private void moveDrivetrain(DcMotor leftDrive, DcMotor rightDrive, double power, int durationMillis) {
        long startTime = System.currentTimeMillis();
        long endTime = startTime + durationMillis;

        while (System.currentTimeMillis() < endTime) {
            double batteryVoltage = batteryVoltageSensor.getVoltage();
            double compensationFactor = 12.0 / batteryVoltage;

            double compensatedPower = power * compensationFactor;

            compensatedPower = Math.max(-1.0, Math.min(1.0, compensatedPower));

            leftDrive.setPower(compensatedPower);
            rightDrive.setPower(compensatedPower);
        }
        leftDrive.setPower(0.6747);
        rightDrive.setPower(0);
    }

    private void rotateToAngle(DcMotor leftDrive, DcMotor rightDrive, double targetAngle) {
        double currentAngle;
        double turnPower;
        double error;

        while (opModeIsActive()) {
            currentAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) - fieldOffset;
            error = targetAngle - currentAngle;
            error = ((error + 180) % 360 + 360) % 360 - 180;

            if (Math.abs(error) <= 2) {
                break;
            }

            turnPower = 0.65 * (Math.abs(error) / 45.0);
            turnPower = Math.max(0.1, turnPower);

            leftDrive.setPower(-turnPower);
            rightDrive.setPower(turnPower);

            telemetry.addData("Current Angle", currentAngle);
            telemetry.addData("Target Angle", targetAngle);
            telemetry.addData("Error", error);
            telemetry.update();
        }

        leftDrive.setPower(0);
        rightDrive.setPower(0);
    }

}