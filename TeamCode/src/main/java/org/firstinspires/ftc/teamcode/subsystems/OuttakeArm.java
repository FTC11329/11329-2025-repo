package org.firstinspires.ftc.teamcode.subsystems;


import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.utility.RobotSideEnum;


public class OuttakeArm {
    Servo clawServo;
    double lastClawPos = 0;

    Servo wristServo;
    double lastWristPos = 0;

    Servo armServo1;
    Servo armServo2;
    double lastArmPos = Constants.Outtake.initTeleopArm;

    RevColorSensorV3 clawSensor;

    HardwareMap hardwareMap;

    RobotSideEnum robotSide;

    public OuttakeArm(HardwareMap hardwareMap, RobotSideEnum robotSide, boolean initArm) {
        this.hardwareMap = hardwareMap;
        clawSensor = hardwareMap.get(RevColorSensorV3.class, "clawSensor");
        clawServo = hardwareMap.get(Servo.class, "clawServo");
        clawServo.setDirection(Servo.Direction.FORWARD);
        clawServo.setPosition(Constants.Outtake.grabClaw);
        if (initArm) {
            initArm();
        }
        this.robotSide = robotSide;
    }

    public void initArm() {
        armServo1 = hardwareMap.get(Servo.class, "armServo1");
        armServo2 = hardwareMap.get(Servo.class, "armServo2");

        armServo1.setDirection(Servo.Direction.FORWARD);
        armServo2.setDirection(Servo.Direction.FORWARD);

        armServo1.setPosition(Constants.Outtake.initTeleopArm);
        armServo2.setPosition(Constants.Outtake.initTeleopArm);

        wristServo = hardwareMap.get(Servo.class, "wrist");

        wristServo.setDirection(Servo.Direction.FORWARD);

        wristServo.setPosition(Constants.Outtake.initTeleopWrist);
    }


    public void manualArmPos(double power) {
        double temp = lastArmPos + (power * Constants.Outtake.manualArmSpeed);
        setArmPos(temp);
    }

    public void setArmPos(double newArmPos) {
        if (lastArmPos != newArmPos) {
            if (newArmPos < Constants.Outtake.initAutoNearWallArm) {
                newArmPos = Constants.Outtake.initAutoNearWallArm;
            } else if (newArmPos > 1) {
                newArmPos = 1;
            }
            lastArmPos = newArmPos;
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

    public void manualWristPos(double power) {
        double temp = lastWristPos + (power * Constants.Outtake.manualWristSpeed);
        setWristPos(temp);
    }
    public void setWristPos(double newWristPos) {
        if (lastWristPos != newWristPos) {
            if (newWristPos < Constants.Outtake.minWrist) {
                newWristPos = Constants.Outtake.minWrist;
            } else if (newWristPos > Constants.Outtake.maxWrist) {
                newWristPos = Constants.Outtake.maxWrist;
            }
            lastWristPos  = newWristPos;
            wristServo.setPosition(newWristPos);
        }
    }

    public double getClawPos() {
        return clawServo.getPosition();
    }

    public double getWristPos() {
        return lastWristPos;
    }
    public double getSensorDistance() {
        return clawSensor.getDistance(DistanceUnit.INCH);
    }

    public boolean seesWall() {
        if (robotSide == RobotSideEnum.Red) {
            return getSensorDistance() < Constants.Outtake.seesWallDistanceRed;
        } else {
            return getSensorDistance() < Constants.Outtake.seesWallDistanceBlue;
        }
    }
    public boolean seesTransfer() {
        return getSensorDistance() < Constants.Outtake.seesTransferDistance;
    }
}