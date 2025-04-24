package org.firstinspires.ftc.teamcode.Main;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Subsystems.AmpMonitor;

@TeleOp(name = "Tank Drive", group = "Drive")
public class MainDrive extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor leftDrive = hardwareMap.dcMotor.get("leftDrive");
        DcMotor rightDrive = hardwareMap.dcMotor.get("rightDrive");
        final DcMotorEx arm = hardwareMap.get(DcMotorEx.class, "arm");

        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        AmpMonitor ampMonitor = new AmpMonitor(arm);

        waitForStart();

        while (opModeIsActive()) {

            double forward = -gamepad1.left_stick_y;
            double turn = gamepad1.right_stick_x;
            double aF = gamepad1.right_trigger;
            double aR = gamepad1.left_trigger * -1;

            double leftPower = turn + forward;
            double rightPower = turn - forward;
            double armPower = aF + aR;

            arm.setPower(armPower);
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