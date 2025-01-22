package org.firstinspires.ftc.teamcode.subsystems;


import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Constants;


public class PowerTakeOff {
    public Servo PTOLeft;
    double lastLeftPos = 0;

    public Servo PTORight;
    double lastRightPos = 0;

    public Servo hookReleaseL;
    double lastReleaseLPos = 0;
    public Servo hookReleaseR;
    double lastReleaseRPos = 0;

    boolean enabled = false;
    public PowerTakeOff(HardwareMap hardwareMap) {
        PTOLeft = hardwareMap.get(Servo.class, "PTOL");
        PTORight = hardwareMap.get(Servo.class, "PTOR");

        PTOLeft.setDirection(Servo.Direction.FORWARD);
        PTORight.setDirection(Servo.Direction.FORWARD);

        PTOLeft.setPosition(Constants.PTO.PTOServoRelease);
        PTORight.setPosition(Constants.PTO.PTOServoRelease);

        hookReleaseL = hardwareMap.get(Servo.class, "hookReleaseL");
        hookReleaseR = hardwareMap.get(Servo.class, "hookReleaseR");

        hookReleaseL.setDirection(Servo.Direction.FORWARD);
        hookReleaseR.setDirection(Servo.Direction.FORWARD);

        hookReleaseL.setPosition(Constants.PTO.grabTheHooksL);
        hookReleaseR.setPosition(Constants.PTO.grabTheHooksR);

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

    public void hookRelease() {
        setHookReleaseL(Constants.PTO.releaseTheHooksL);
        setHookReleaseR(Constants.PTO.releaseTheHooksR);
    }
    public void hookGrab() {
        setHookReleaseL(Constants.PTO.grabTheHooksL);
        setHookReleaseR(Constants.PTO.grabTheHooksR);
    }
    public void setHookReleaseL(double newPos) {
        if (lastReleaseLPos != newPos) {
            lastReleaseLPos = newPos;
            hookReleaseL.setPosition(newPos);
        }
    }
    public void setHookReleaseR(double newPos) {
        if (lastReleaseRPos != newPos) {
            lastReleaseRPos = newPos;
            hookReleaseR.setPosition(newPos);
        }
    }
}