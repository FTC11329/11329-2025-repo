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

        PTOLeft.setPosition(Constants.PTO.PTOServoRelease);
        PTORight.setPosition(Constants.PTO.PTOServoRelease);
    }

    public void enable() {
        setRightPos(Constants.PTO.PTOServoClimb);
        setLeftPos(Constants.PTO.PTOServoClimb);
        enabled = true;
    }
    public void disable() {
        setRightPos(Constants.PTO.PTOServoRelease);
        setLeftPos(Constants.PTO.PTOServoRelease);
        enabled = false;
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