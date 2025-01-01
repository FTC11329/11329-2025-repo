package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.Constants;

public class VerticalSlides {

    DcMotorEx slideMotor;
    int lastSlidePos;

    boolean lastPressed = false;
    boolean touched = false;

    public TouchSensor touchSensor;

    public VerticalSlides(HardwareMap hardwareMap) {
        slideMotor = hardwareMap.get(DcMotorEx.class, "vSlides");

        touchSensor = hardwareMap.get(TouchSensor.class, "LiftMagnetSensor");

        slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotor.setTargetPosition(0);
        slideMotor.setPower(1);
        slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void manualPos(double power) {
        int temp = lastSlidePos + (int)(power * Constants.Outtake.manualSlideSpeed);
        if (temp < Constants.Outtake.maxSlides) {
            setPos(temp);
        }
    }

    public void setPos(int newPos) {
        if (lastSlidePos != newPos) {
            lastSlidePos  = newPos;
            slideMotor.setTargetPosition(newPos);
        }
    }

    public int getTargetPos() {
        return lastSlidePos;
    }
    public int getPos() {
        return slideMotor.getCurrentPosition();
    }


    public boolean nearlyTuchyWuchyed() {
        return touchSensor.isPressed();
    }

    public void update() {
        touched = touchSensor.isPressed();
        if (touched && !lastPressed) {
            slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        lastPressed = touchSensor.isPressed();
    }
}
