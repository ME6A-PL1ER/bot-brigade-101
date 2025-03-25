package org.firstinspires.ftc.teamcode.Main;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp
public class MainDrive extends LinearOpMode {
    private DcMotor leftDrive;
    private DcMotor rightDrive;
    private IMU imu;
    double fieldOffset;

    private boolean isHalfSpeedMode = false;

    double xOffset = 0.089;
    double yOffset = -0.0508;

    @Override
    public void runOpMode() throws InterruptedException {
        leftDrive = hardwareMap.dcMotor.get("leftDrive");
        rightDrive = hardwareMap.dcMotor.get("rightDrive");
        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT));
        imu.initialize(parameters);

        waitForStart();
        if (isStopRequested()) return;

        telemetry.addData("Status: ", "initialized");
        telemetry.update();

        while (opModeIsActive()) {
            double x = gamepad1.left_stick_x;
            double y = gamepad1.left_stick_y;
            double rx = gamepad1.right_stick_x;

            if (gamepad1.right_stick_button) {
                isHalfSpeedMode = true;
            } else {
                isHalfSpeedMode = false;
            }

            if (gamepad1.y) {
                fieldOffset = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            }

            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) - fieldOffset;
            double headingRad = Math.toRadians(botHeading);

            double tempX = (x * Math.cos(headingRad) + y * Math.sin(headingRad)) - xOffset;
            double tempY = (-x * Math.sin(headingRad) + y * Math.cos(headingRad)) - yOffset;

            double speedMultiplier = isHalfSpeedMode ? 0.3 : 1.0;

            double leftPower = tempY + rx;
            double rightPower = tempY - rx;

            leftPower *= speedMultiplier;
            rightPower *= speedMultiplier;

            leftDrive.setPower(leftPower);
            rightDrive.setPower(rightPower);

            telemetry.addData("Status", "Running");
            telemetry.addData("Speed Mode", isHalfSpeedMode ? "Half-Speed" : "Full-Speed");
            telemetry.addData("Left Power", leftPower);
            telemetry.addData("Right Power", rightPower);
            telemetry.addData("IMU Yaw", botHeading);
            telemetry.update();
        }
    }
}
