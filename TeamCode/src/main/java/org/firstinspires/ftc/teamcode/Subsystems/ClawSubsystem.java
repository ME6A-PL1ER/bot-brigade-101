package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

public class ClawSubsystem extends SubsystemBase {
    private final Servo servo;
    private final CRServo subServo;

    public ClawSubsystem(Servo servo, CRServo subServo) {
        this.servo = servo;
        this.subServo = subServo;
    }


    public void setMainServoPosition(double position) {
        position = Math.max(-90, Math.min(90, position));

        double mappedPosition = (position + 90) / 180.0;

        servo.setPosition(mappedPosition);
    }

    public void setSubServoPosition(double subPosition) {
    }

}
