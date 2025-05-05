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


@Autonomous(name = "2 banger park", group = "autos")
public class TwoSpeciminePark extends LinearOpMode {
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

        armSubsystem.autoArmMover(4220);
        clawSubsystem.setServoPosition(45);
        autoSubsystem.move(leftDrive, rightDrive, -0.5, 700);
        autoSubsystem.move(leftDrive, rightDrive, -0.2, 900);
        sleep(500);
        clawSubsystem.setServoPosition(0);
        sleep(500);
        // Yoink first^^
        autoSubsystem.move(leftDrive, rightDrive, 0.1, 200);
        armSubsystem.autoArmMover(3700);
        // Now off the wall
        autoSubsystem.move(leftDrive, rightDrive, 0.5, 1500);
        rotateToAngle(leftDrive, rightDrive, 90);
        // Ready to go forward and place
        armSubsystem.autoArmMover(3400);
        autoSubsystem.move(leftDrive, rightDrive, -0.5, 1400);
        armSubsystem.autoArmMover(3900);
        // Placed
        clawSubsystem.setServoPosition(45);
        autoSubsystem.move(leftDrive, rightDrive, 0.3, 500);
        rotateToAngle(leftDrive, rightDrive, 0);
        autoSubsystem.move(leftDrive, rightDrive, -0.8, 900);
        rotateToAngle(leftDrive, rightDrive, -90);
        // Looking at wall human player
        armSubsystem.autoArmMover(4250);
        clawSubsystem.setServoPosition(-45);
        sleep(500);
        autoSubsystem.move(leftDrive, rightDrive, -0.3, 1750);
        sleep(1000);
        clawSubsystem.setServoPosition(0);
        sleep(750);
        // Yoink 2nd
        autoSubsystem.move(leftDrive, rightDrive, 0.1, 200);
        armSubsystem.autoArmMover(3700);
        // Off the wall
        autoSubsystem.move(leftDrive, rightDrive, 0.3, 200);
        rotateToAngle(leftDrive, rightDrive, 0);
        autoSubsystem.move(leftDrive, rightDrive, 0.5, 1300);
        rotateToAngle(leftDrive, rightDrive, 90);
        // Ready to go forward and place
        armSubsystem.autoArmMover(3400);
        autoSubsystem.move(leftDrive, rightDrive, -0.5, 750);
        armSubsystem.autoArmMover(3900);
        // Placed
        clawSubsystem.setServoPosition(45);
        autoSubsystem.move(leftDrive, rightDrive, 0.3, 700);
        rotateToAngle(leftDrive, rightDrive, -10);
        armSubsystem.autoArmMover(0);
        autoSubsystem.move(leftDrive, rightDrive, -1, 1000);
    }

    public void rotateToAngle(DcMotor leftDrive, DcMotor rightDrive, double targetAngle) {
        double kP = 0.05;  // Adjusts speed
        double kI = 0; // Don't touch it #1
        double kD = 0;  // Don't touch it #2

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