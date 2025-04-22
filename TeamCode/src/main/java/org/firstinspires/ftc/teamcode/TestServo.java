//package org.firstinspires.ftc.teamcode;
//
//import com.arcrobotics.ftclib.command.SubsystemBase;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.Servo;
//
//public class TestServo extends LinearOpMode {
//    private final Servo servo;
//    private boolean servoContinuous;
//
//    public TestServo(Servo servo) {
//        this.servo = servo;
//    }
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//        final Servo slideServo = hardwareMap.get(Servo.class, "servo");
//
//        waitForStart();
//        if (isStopRequested()) return;
//
//        while (opModeIsActive()) {
//            if (gamepad1.y && servoContinuous ! true) {
//                servoContinuous = true;
//            } else if (gamp)
//
//            if (servoContinuous == false) {
//                if (gamepad1.a) {
//                    servo.setPosition(0);
//                }
//
//                if (gamepad1.x) {
//                    servo.setPosition(-90);
//                }
//
//                if (gamepad1.b) {
//                    servo.setPosition(90);
//                }
//            }
//        }
//    }
//}
