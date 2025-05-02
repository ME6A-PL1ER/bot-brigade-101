package org.firstinspires.ftc.teamcode.Main;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Subsystems.ClawSubsystem;

import org.firstinspires.ftc.teamcode.Subsystems.AmpMonitor;

@TeleOp(name = "ඞ pick this one ඞ", group = "Drive")
public class MainDrive extends LinearOpMode {

    boolean climbing = false;

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor leftDrive = hardwareMap.dcMotor.get("leftDrive");
        DcMotor rightDrive = hardwareMap.dcMotor.get("rightDrive");
        final DcMotorEx arm = hardwareMap.get(DcMotorEx.class, "arm");
        Servo servo = hardwareMap.get(Servo.class, "servo");

        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        AmpMonitor ampMonitor = new AmpMonitor(arm);
        ClawSubsystem clawSubsystem = new ClawSubsystem(servo);


        clawSubsystem.setServoPosition(0);
        waitForStart();

        while (opModeIsActive()) {

            double forward = -gamepad1.left_stick_y;
            double turn = gamepad1.right_stick_x * 0.6;
            double aF = gamepad1.right_trigger;
            double aR = gamepad1.left_trigger * -1;

            double leftPower = turn + forward;
            double rightPower = turn - forward;
            double armPower = aF + aR;

            if (gamepad1.a) {clawSubsystem.setServoPosition(45);}
            if (gamepad1.x) {clawSubsystem.setServoPosition(0);}

            if (gamepad1.left_bumper) {climbing = false;}
            if (gamepad1.right_bumper) {climbing = true;}

            if (climbing) {arm.setPower(-1);}
            else {arm.setPower(armPower);}
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