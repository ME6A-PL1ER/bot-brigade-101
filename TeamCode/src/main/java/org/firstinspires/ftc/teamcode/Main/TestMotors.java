package org.firstinspires.ftc.teamcode.Main;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Test Motors", group = "Drive")
public class TestMotors extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor leftDrive = hardwareMap.dcMotor.get("leftDrive");
        DcMotor rightDrive = hardwareMap.dcMotor.get("rightDrive");
        DcMotor arm = hardwareMap.dcMotor.get("arm");

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            leftDrive.setPower(-gamepad1.left_stick_y);
            rightDrive.setPower(gamepad1.right_stick_y);
            arm.setPower(gamepad1.left_stick_x);
        }
    }
}