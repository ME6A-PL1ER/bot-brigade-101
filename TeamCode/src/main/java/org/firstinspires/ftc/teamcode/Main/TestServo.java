package org.firstinspires.ftc.teamcode.Main;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Subsystems.ClawSubsystem;

import org.firstinspires.ftc.teamcode.Subsystems.AmpMonitor;

@TeleOp(name = "test servo", group = "Drive")
public class TestServo extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        CRServo intake = hardwareMap.get(CRServo.class, "intake");

        waitForStart();
        while (opModeIsActive()) {

            double iF = gamepad1.right_trigger;
            double iR = gamepad1.left_trigger;
            intake.setPower(iF + iR);
        }
    }
}