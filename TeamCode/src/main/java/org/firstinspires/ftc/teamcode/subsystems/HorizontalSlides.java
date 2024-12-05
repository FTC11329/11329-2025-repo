package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Constants;

public class HorizontalSlides {

    DcMotorEx slideMotor;
    int lastSlidePos;

    public HorizontalSlides(HardwareMap hardwareMap) {
        slideMotor = hardwareMap.get(DcMotorEx.class, "hSlides");

        slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotor.setTargetPosition(0);
        slideMotor.setPower(1);
        slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideMotor.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public void manualPos(double power) {
        int temp = lastSlidePos + (int)(power * Constants.Intake.manualSlideSpeed);
        if (0 < temp && temp < Constants.Intake.maxSlidePos) {
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



}
