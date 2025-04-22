package org.firstinspires.ftc.teamcode.Autos;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import org.firstinspires.ftc.teamcode.Subsystems.AutoSubsystem;


@Autonomous(name = "Just drive forward", group = "autos")
public class JustDriveForward extends LinearOpMode {
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

        autoSubsystem.move(leftDrive, rightDrive, 0.5, 2000);
        return;
    }
}