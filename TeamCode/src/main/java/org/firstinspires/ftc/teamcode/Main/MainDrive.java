package org.firstinspires.ftc.teamcode.Main;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Subsystems.ClawSubsystem;

import org.firstinspires.ftc.teamcode.Subsystems.AmpMonitor;

@TeleOp(name = "ඞ pick this one ඞ", group = "Drive")
public class MainDrive extends LinearOpMode {

    boolean climbing = false;
    private int presetCycle;
    private int lastPresetCycle;
    double intakePower;

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor leftDrive = hardwareMap.dcMotor.get("leftDrive");
        DcMotor rightDrive = hardwareMap.dcMotor.get("rightDrive");
        final DcMotorEx arm = hardwareMap.get(DcMotorEx.class, "arm");
        Servo servo = hardwareMap.get(Servo.class, "servo");
        CRServo intake = hardwareMap.get(CRServo.class, "intake");

        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        AmpMonitor ampMonitor = new AmpMonitor(arm);
        ClawSubsystem clawSubsystem = new ClawSubsystem(servo);


        clawSubsystem.setMainServoPosition(0);
        waitForStart();

        while (opModeIsActive()) {

            double forward = -gamepad1.left_stick_y;
            double turn = gamepad1.right_stick_x;
            double aF = gamepad1.right_trigger * 0.75;
            double aR = gamepad1.left_trigger * -0.75;

            if (presetCycle != lastPresetCycle) {
                switch (presetCycle) {
                    case 1:
                        clawSubsystem.setMainServoPosition(0);
                        break;
                    case 2:
                        clawSubsystem.setMainServoPosition(45);
                        break;
                    default:
                        telemetry.addData("preset", "none selected");
                        telemetry.update();
                        break;
                }
                lastPresetCycle = presetCycle;
            }

            if (gamepad1.left_bumper) {climbing = false;}
            if (gamepad1.right_bumper) {climbing = true;}

            if (gamepad1.dpad_left) {turn = -0.2;}
            if (gamepad1.dpad_right) {turn = 0.2;}

            if (gamepad1.y) {intakePower = 1.0;}
            if (gamepad1.b) {intakePower = -1.0;}

            double leftPower = turn + forward;
            double rightPower = turn - forward;
            double armPower = aF + aR;

            if (climbing) {arm.setPower(-1);}
            else {arm.setPower(armPower);}

            intake.setPower(intakePower);
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