package org.firstinspires.ftc.teamcode.Main;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Subsystems.ArmSubsystem;

@TeleOp(name = "Tank Drive", group = "Drive")
public class MainDrive extends LinearOpMode {

    double armTarget;

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor leftDrive = hardwareMap.dcMotor.get("leftDrive");
        DcMotor rightDrive = hardwareMap.dcMotor.get("rightDrive");
        DcMotor arm = hardwareMap.dcMotor.get("arm");

        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            double forward = -gamepad1.left_stick_y;
            double turn = gamepad1.right_stick_x;

            double leftPower = turn + forward;
            double rightPower = turn - forward;

            if (gamepad1.dpad_up) {arm.setPower(0.7);}
            else if (gamepad1.dpad_down) {arm.setPower(-0.7);}
            else {arm.setPower(0);}

            leftDrive.setPower(leftPower);
            rightDrive.setPower(rightPower);

            telemetry.addData("Left Power", leftPower);
            telemetry.addData("Right Power", rightPower);
            telemetry.addData("L stick up/down", gamepad1.left_stick_y);
            telemetry.addData("R stick left/right", gamepad1.right_stick_x);
            telemetry.update();
        }
    }
}