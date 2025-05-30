package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class AutoSubsystem extends SubsystemBase {

    private DcMotor leftDrive;
    private DcMotor rightDrive;
    private VoltageSensor batteryVoltageSensor;
    private IMU imu;
    private double fieldOffset = 0;
    private double currentRotation;


    public AutoSubsystem(HardwareMap hardwareMap) {
        leftDrive = hardwareMap.dcMotor.get("leftDrive");
        rightDrive = hardwareMap.dcMotor.get("rightDrive");
        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT));
        imu.initialize(parameters);

        batteryVoltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");
    }

    public void move(DcMotor leftDrive, DcMotor rightDrive, double power, int durationMillis) {
        long startTime = System.currentTimeMillis();
        long endTime = startTime + durationMillis;

        while (System.currentTimeMillis() < endTime) {
            double batteryVoltage = batteryVoltageSensor.getVoltage();
            double compensationFactor = 12.0 / batteryVoltage;

            double compensatedPower = power * compensationFactor;

            compensatedPower = Math.max(-1.0, Math.min(1.0, compensatedPower));

            leftDrive.setPower(-compensatedPower);
            rightDrive.setPower(compensatedPower);
        }
        leftDrive.setPower(0);
        rightDrive.setPower(0);
    }
}
