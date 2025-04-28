package org.firstinspires.ftc.teamcode.Autos;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Subsystems.AutoSubsystem;


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
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        imu.initialize(parameters);

        batteryVoltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");

        AutoSubsystem autoSubsystem = new AutoSubsystem(hardwareMap);

        telemetry.addData("Status", "Initialized. Waiting for start...");
        telemetry.update();
        waitForStart();
        if (isStopRequested()) return;

        /*
            Bot should be started with omni wheels facing forward and the left wheels on the seam of
            the basket tile and the adjacent one to the right
            The degrees might be reversed so what we think would be 90 degrees is really -90
            (Instead of the circle being drawn clockwise its counterclockwise)
         */

        autoSubsystem.move(leftDrive, rightDrive, 0.5, 750);

        rotateToAngle(leftDrive, rightDrive, -45);

        autoSubsystem.move(leftDrive, rightDrive, 0.5, 150);

        rotateToAngle(leftDrive, rightDrive, -165);

        autoSubsystem.move(leftDrive, rightDrive, 0.5, 600);

        autoSubsystem.move(leftDrive, rightDrive, -0.5, 600);

        rotateToAngle(leftDrive, rightDrive, -90);

        autoSubsystem.move(leftDrive, rightDrive, 0.5, 250);

        rotateToAngle(leftDrive, rightDrive, -172);

        autoSubsystem.move(leftDrive, rightDrive, -0.5, 300);

        rotateToAngle(leftDrive, rightDrive, 103);

        autoSubsystem.move(leftDrive, rightDrive, 1, 3000);
    }

    private void rotateToAngle(DcMotor leftDrive, DcMotor rightDrive, double targetAngle) {
        imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        double currentAngle;
        double turnPower;
        double error;
        double absError;

        while (opModeIsActive()) {
            currentAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

            error = targetAngle - currentAngle;
            error = ((error + 180) % 360 + 360) % 360 - 180;
            absError = Math.abs(error);

            if (absError < 2) {
                break;
            }

            turnPower = 0.65 * (absError / 45.0);
            turnPower = Math.max(0.1, turnPower);

            if (error > 0) {
                leftDrive.setPower(turnPower);
                rightDrive.setPower(turnPower);
            }

            telemetry.addData("Current Angle", currentAngle);
            telemetry.addData("Target Angle", targetAngle);
            telemetry.addData("Error", error);
            telemetry.update();
        }
        leftDrive.setPower(0);
        rightDrive.setPower(0);
    }
}