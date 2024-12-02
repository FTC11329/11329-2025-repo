package org.firstinspires.ftc.teamcode.subsystems;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;


public class OuttakeArm {
    Servo clawServo;
    double lastClawPos = 0;

    Servo armServo1;
    Servo armServo2;
    double lastArmPos = 0;

    public OuttakeArm(HardwareMap hardwareMap) {
        clawServo = hardwareMap.get(Servo.class, "clawServo");
        armServo1 = hardwareMap.get(Servo.class, "armServo1");
        armServo2 = hardwareMap.get(Servo.class, "armServo2");

        clawServo.setDirection(Servo.Direction.FORWARD);
        armServo1.setDirection(Servo.Direction.FORWARD);
        armServo2.setDirection(Servo.Direction.FORWARD);

        clawServo.setPosition(Constants.Outtake.grabClaw);
        armServo1.setPosition(Constants.Outtake.initArm);
        armServo2.setPosition(Constants.Outtake.initArm);

    }

    public void setArmPos(double newArmPos) {
        if (lastArmPos != newArmPos) {
            lastArmPos  = newArmPos;
            armServo1.setPosition(newArmPos);
            armServo2.setPosition(newArmPos);
        }
    }

    public void setClawPos(double newClawPos) {
        if (lastClawPos != newClawPos) {
            lastClawPos  = newClawPos;
            clawServo.setPosition(newClawPos);
        }
    }
    public double getClawPos () {
        return clawServo.getPosition();
    }
}