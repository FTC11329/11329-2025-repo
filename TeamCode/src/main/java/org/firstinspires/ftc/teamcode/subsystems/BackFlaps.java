package org.firstinspires.ftc.teamcode.subsystems;

import android.graphics.Bitmap;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Constants;

public class BackFlaps {
    Servo leftFlap;
    double lastLeftFlapPos = 0;
    Servo rightFlap;
    double lastRightFlapPos = 0;
    public BackFlaps(HardwareMap hardwareMap) {
        leftFlap  = hardwareMap.get(Servo.class, "leftFlap" );
//        rightFlap = hardwareMap.get(Servo.class, "rightFlap");

        leftFlap.setDirection(Servo.Direction.FORWARD);
//        rightFlap.setDirection(Servo.Direction.FORWARD);

        setFlapsUp();
    }

    public void setFlapsUp() {
        setLeftFlap(Constants.Outtake.leftFlapUp);
        setRightFlap(Constants.Outtake.rightFlapUp);
    }
    public void setFlapsDown() {
        setLeftFlap(Constants.Outtake.leftFlapWall);
        setRightFlap(Constants.Outtake.rightFlapWall);
    }

    public void setLeftFlap(double set) {
        if (lastLeftFlapPos != set) {
            lastLeftFlapPos  = set;
            leftFlap.setPosition(set);
        }
    }

    public void setRightFlap(double set) {
        if (lastRightFlapPos != set) {
            lastRightFlapPos  = set;
//            rightFlap.setPosition(set);
        }
    }
}
