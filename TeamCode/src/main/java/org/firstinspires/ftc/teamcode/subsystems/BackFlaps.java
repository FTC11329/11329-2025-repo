package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Constants;

public class BackFlaps {
    Servo backFlaps;
    double lastBackFlapPos = 1;
    public BackFlaps(HardwareMap hardwareMap) {
        backFlaps = hardwareMap.get(Servo.class, "backFlap");

        backFlaps.setDirection(Servo.Direction.FORWARD);

        setFlapsUp();
    }

    public void setFlapsUp() {
        setBackFlaps(Constants.Outtake.flapsUp);
    }
    public void setFlapsWall() {
        setBackFlaps(Constants.Outtake.flapsWall);
    }

    public void setFlapsSpikeClear() {
        setBackFlaps(Constants.Outtake.flapsSpikeClear);
    }

    public void setBackFlaps(double set) {
        if (lastBackFlapPos != set) {
            lastBackFlapPos = set;
            backFlaps.setPosition(set);
        }
    }

    public double getPos() {
        return lastBackFlapPos;
    }
}
