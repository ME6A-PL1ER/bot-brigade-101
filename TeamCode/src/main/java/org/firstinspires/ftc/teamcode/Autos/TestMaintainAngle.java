package org.firstinspires.ftc.teamcode.Autos;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.AutoSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.ClawSubsystem;


@Autonomous(name = "test angle management", group = "autos")
public class TestMaintainAngle extends LinearOpMode {
    private DcMotor leftDrive;
    private DcMotor rightDrive;
    private VoltageSensor batteryVoltageSensor;
    private IMU imu;
    private double fieldOffset = 0;
    private boolean timerStarted;
    ElapsedTime stabilityTimer = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor leftDrive = hardwareMap.dcMotor.get("leftDrive");
        DcMotor rightDrive = hardwareMap.dcMotor.get("rightDrive");
        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Servo servo = hardwareMap.get(Servo.class, "servo");

        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        imu.initialize(parameters);

        batteryVoltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");

        AutoSubsystem autoSubsystem = new AutoSubsystem(hardwareMap);
        ArmSubsystem armSubsystem = new ArmSubsystem(hardwareMap);
        ClawSubsystem clawSubsystem = new ClawSubsystem(servo);

        imu.resetYaw();
        armSubsystem.resetZero();

        telemetry.addData("Status", "Initialized. Waiting for start...");
        telemetry.update();
        waitForStart();
        if (isStopRequested()) return;

        moveMaintainAngle(leftDrive, rightDrive, 0.5, 5000);
    }

    public void rotateToAngle(DcMotor leftDrive, DcMotor rightDrive, double targetAngle) {
        double kP = 0.11;  // Adjusts speed
        double kI = 0; // Don't touch it
        double kD = 1;  // Don't overcompensate

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

    public void moveMaintainAngle(DcMotor leftDrive, DcMotor rightDrive, double power, int durationMillis) {
        double kP = 0.05;
        double kI = 0;
        double kD = 0.01;

        double initialAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

        double error;
        double lastError = 0;
        double integral = 0;
        double derivative;
        double correction;

        long startTime = System.currentTimeMillis();
        long endTime = startTime + durationMillis;

        while (System.currentTimeMillis() < endTime && opModeIsActive()) {
            double currentAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

            error = initialAngle - currentAngle;
            if (error > 180) error -= 360;
            if (error < -180) error += 360;

            integral += error;
            derivative = error - lastError;
            lastError = error;

            correction = (kP * error) + (kI * integral) + (kD * derivative);

            double batteryVoltage = batteryVoltageSensor.getVoltage();
            double compensationFactor = 12.0 / batteryVoltage;
            double compensatedPower = power * compensationFactor;
            compensatedPower = Math.max(-1.0, Math.min(1.0, compensatedPower));

            double leftPower = -compensatedPower - correction;
            double rightPower = compensatedPower - correction;

            leftPower = Math.max(-1.0, Math.min(1.0, leftPower));
            rightPower = Math.max(-1.0, Math.min(1.0, rightPower));

            leftDrive.setPower(leftPower);
            rightDrive.setPower(rightPower);
        }

        leftDrive.setPower(0);
        rightDrive.setPower(0);
    }
}