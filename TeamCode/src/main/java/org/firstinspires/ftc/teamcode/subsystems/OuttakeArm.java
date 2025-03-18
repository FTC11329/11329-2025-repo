package org.firstinspires.ftc.teamcode.subsystems;


import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Constants;


public class OuttakeArm {
    Servo clawServo;
    double lastClawPos = 0;

    Servo armServo1;
    Servo armServo2;
    double lastArmPos = Constants.Outtake.initTeleopArm;

    HardwareMap hardwareMap;

    public OuttakeArm(HardwareMap hardwareMap, boolean initArm) {
        this.hardwareMap = hardwareMap;
        clawServo = hardwareMap.get(Servo.class, "clawServo");
        clawServo.setDirection(Servo.Direction.FORWARD);
        clawServo.setPosition(Constants.Outtake.grabClaw);
        if (initArm) {
            initArm();
        }
    }

    public void initArm() {
        armServo1 = hardwareMap.get(Servo.class, "armServo1");
        armServo2 = hardwareMap.get(Servo.class, "armServo2");

        armServo1.setDirection(Servo.Direction.FORWARD);
        armServo2.setDirection(Servo.Direction.FORWARD);

        armServo1.setPosition(Constants.Outtake.initTeleopArm);
        armServo2.setPosition(Constants.Outtake.initTeleopArm);
    }

    public void manualArmPos(double power) {
        double temp = lastArmPos + (power * Constants.Outtake.manualArmSpeed);
        setArmPos(temp);
    }
    public void setArmPos(double newArmPos) {
        if (lastArmPos != newArmPos) {
            lastArmPos  = newArmPos;
            armServo1.setPosition(newArmPos);
            armServo2.setPosition(newArmPos);
        }
    }
    public double getArmPos() {
        return lastArmPos;
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