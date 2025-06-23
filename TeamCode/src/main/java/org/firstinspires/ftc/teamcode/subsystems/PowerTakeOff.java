package org.firstinspires.ftc.teamcode.subsystems;


import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Constants;


public class PowerTakeOff {
    public Servo PTOLeft;
    double lastLeftPos = 0;

    public Servo PTORight;
    double lastRightPos = 0;

    boolean enabled = false;
    public PowerTakeOff(HardwareMap hardwareMap) {
        PTOLeft = hardwareMap.get(Servo.class, "PTOL");
        PTORight = hardwareMap.get(Servo.class, "PTOR");

        PTOLeft.setDirection(Servo.Direction.FORWARD);
        PTORight.setDirection(Servo.Direction.FORWARD);

        PTORight.setPosition(Constants.PTO.PTOServoReleaseRight);
        PTOLeft.setPosition(Constants.PTO.PTOServoReleaseLeft);
    }

    public void enable() {
        setRightPos(Constants.PTO.PTOServoClimbRight);
        setLeftPos(Constants.PTO.PTOServoClimbLeft);
        enabled = true;
    }
    public void disable() {
        setRightPos(Constants.PTO.PTOServoReleaseRight);
        setLeftPos(Constants.PTO.PTOServoReleaseLeft);
        enabled = false;
    }

    public void hold() {
        setRightPos(Constants.PTO.PTOServoClimbRight);
        setLeftPos(Constants.PTO.PTOServoClimbLeft);
    }

    //WATER BUCKET,
    public void    release() {
        setRightPos(Constants.PTO.PTOServoReleaseRight);
        setLeftPos(Constants.PTO.PTOServoReleaseLeft);
    }


    public boolean isEnabled() {
        return enabled;
    }

    public void setLeftPos(double newPos) {
        if (lastLeftPos != newPos) {
            lastLeftPos = newPos;
            PTOLeft.setPosition(newPos);
        }
    }

    public void setRightPos(double newPos) {
        if (lastRightPos != newPos) {
            lastRightPos = newPos;
            PTORight.setPosition(newPos);
        }
    }
}