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

        ArmSubsystem armSubsystem = new ArmSubsystem(hardwareMap, arm);
        armSubsystem.resetZero();

        waitForStart();

        while (opModeIsActive()) {

            boolean armModeNormal = false;
            boolean lastButtonState = false;

            if (gamepad1.left_stick_button && !lastButtonState) {
                armModeNormal = !armModeNormal;
            }
            lastButtonState = gamepad1.left_stick_button;

            double forward = -gamepad1.left_stick_y;
            double turn = gamepad1.right_stick_x;

            double leftPower = turn + forward;
            double rightPower = turn - forward;

            if (gamepad1.left_trigger > 0) {armTarget += 20;}
            if (gamepad1.right_trigger > 0) {armTarget -=20;}

            if (gamepad1.right_trigger > 0 && armModeNormal) {arm.setPower(gamepad1.right_trigger);}
            if (gamepad1.left_trigger > 1 && armModeNormal) {arm.setPower(-gamepad1.left_trigger);}

            armSubsystem.setPosition(armTarget);
            if (!armModeNormal) {armSubsystem.update();}
            leftDrive.setPower(leftPower);
            rightDrive.setPower(rightPower);

            telemetry.addData("Left Power", leftPower);
            telemetry.addData("Right Power", rightPower);
            telemetry.addData("L stick up/down", gamepad1.left_stick_y);
            telemetry.addData("R stick left/right", gamepad1.right_stick_x);
            telemetry.addData("Target position", armTarget);
            telemetry.addData("Actual position", armSubsystem.getArmPosition());
            telemetry.addData("Error", armSubsystem.getArmError());
            telemetry.addData("Arm power", armSubsystem.getArmPower());
            telemetry.addData("Arm mode", armModeNormal);
            telemetry.update();
        }
    }
}