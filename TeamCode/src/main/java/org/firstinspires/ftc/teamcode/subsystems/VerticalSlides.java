package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Constants;

public class VerticalSlides {

    ElapsedTime time = new ElapsedTime();
    boolean disabled = false;

    DcMotorEx slideMotor;
    int lastSlidePos;

        double lastPressedTime = 0;
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
        slideMotor.setCurrentAlert(4, CurrentUnit.AMPS);
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

    public double getAmp() {
        return slideMotor.getCurrent(CurrentUnit.AMPS);
    }

    public boolean overAmp() {
        return slideMotor.isOverCurrent();
    }

    public boolean nearlyTuchyWuchyed() {
        return touchSensor.isPressed();
    }

    public void update(boolean limit) {
        touched = touchSensor.isPressed();
        if (disabled) {
            return;
        }
        if (limit) {
            if (touched && time.milliseconds() > lastPressedTime + 100 && lastSlidePos < getPos()) {
                lastPressedTime = time.milliseconds();
                slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slideMotor.setTargetPosition(0);
                lastSlidePos = 0;
            }
        } else {
            if (overAmp() && touched && getTargetPos() < getPos()) {
                // if stalling into the robot
                slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slideMotor.setTargetPosition(0);
            }
        }
    }

    public void disable() {
        slideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        disabled = true;
    }

    public void reEnable(int slidePos) {
        setPos(slidePos);
        slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        disabled = false;
    }
}
