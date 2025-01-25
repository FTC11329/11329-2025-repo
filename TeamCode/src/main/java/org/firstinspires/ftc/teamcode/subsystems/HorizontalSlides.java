package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Constants;

public class HorizontalSlides {

    ElapsedTime time = new ElapsedTime();
    boolean disabled = false;

    DcMotorEx slideMotor;
    int lastSlidePos;

    double lastPressedTime = 0;
    boolean touched = false;

    TouchSensor touchSensor;

    public HorizontalSlides(HardwareMap hardwareMap) {
        slideMotor = hardwareMap.get(DcMotorEx.class, "hSlides");

        touchSensor = hardwareMap.get(TouchSensor.class, "H slide touch sensor");

        slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotor.setTargetPosition(0);
        slideMotor.setPower(1);
        slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideMotor.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public void manualPos(double power) {
        int temp = lastSlidePos + (int)(power * Constants.Intake.manualSlideSpeed);
        if (temp < Constants.Intake.maxSlidePos) {
            setPos(temp);
        }
    }

    public void setPos(int newPos) {
        if (lastSlidePos != newPos && newPos < Constants.Intake.maxSlidePos) {
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

    public boolean tuchyWuchy() {
        return touchSensor.isPressed();
    }

    public void update() {
        touched = touchSensor.isPressed();
        if (disabled) {
            if (!touched) {
                slideMotor.setPower(-0.2);
            }
            return;
        }
        if (touched && time.milliseconds() > lastPressedTime + 100 && lastSlidePos < getPos()) {
            lastPressedTime = time.milliseconds();
            slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slideMotor.setTargetPosition(0);
            lastSlidePos = 0;
        }
    }

    public void disable() {
        slideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        disabled = true;
    }

    public void reEnable(int slidePos) {
        setPos(slidePos);
        slideMotor.setPower(1);
        slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        disabled = false;
    }
}
