package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.pedropathing.util.CustomPIDFCoefficients;
import org.firstinspires.ftc.teamcode.pedropathing.util.PIDFController;

public class VerticalSlides {
    PIDFController pid;

    boolean disabled = false;

    boolean reZeroButton = false;
    boolean reZeroButtonDebounce = true;

    DcMotorEx slideMotor;
    int lastSlidePos = 0;

    public VerticalSlides(HardwareMap hardwareMap) {
        slideMotor = hardwareMap.get(DcMotorEx.class, "vSlides");

        slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotor.setTargetPosition(0);
        slideMotor.setPower(0);
        slideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        slideMotor.setCurrentAlert(4, CurrentUnit.AMPS);
//        slideMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, new PIDFCoefficients(10,0.05, 0, 0)); // og
//        slideMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, new PIDFCoefficients(9.5,0.02, 0.05, 0.05)); // new
        pid = new PIDFController(new CustomPIDFCoefficients(0.01, 0, 0.05, 0.035));
    }

    public void manualPos(double power) {
        int temp = lastSlidePos + (int)(power * Constants.Outtake.manualSlideSpeed);
        setPos(temp);
    }

    public void setPos(int newPos) {
        if (!reZeroButton && lastSlidePos != newPos) {
            if (newPos > Constants.Outtake.maxSlides) {
                newPos = Constants.Outtake.maxSlides;
            } else if (newPos < 5) {
                newPos = 5;
            }
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

    public void update(boolean reZeroButton) {
        this.reZeroButton = reZeroButton;
        if (reZeroButton && reZeroButtonDebounce) {
            slideMotor.setPower(-0.5);
            reZeroButtonDebounce = false;
        }
        if (!reZeroButton && !reZeroButtonDebounce) {
            slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            slideMotor.setTargetPosition(5);
            lastSlidePos = 5;
            slideMotor.setPower(0);
            slideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            reZeroButtonDebounce = true;
        }
        if (!reZeroButton) {
            pid.setTargetPosition(lastSlidePos);
            pid.updatePosition(slideMotor.getCurrentPosition());
            slideMotor.setPower(pid.runPIDF());
        }
    }

    public void disable() {
        slideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slideMotor.setPower(0);
        disabled = true;
    }

    public void reEnable(int slidePos) {
        setPos(slidePos);
        slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        disabled = false;
    }
}
