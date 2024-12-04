package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.utility.SimplePIDControl;

public class VerticalSlides {

    DcMotorEx slideMotor;
    int lastSlidePos;

    DcMotorEx encoderSlave;

    SimplePIDControl pidControl;

    public VerticalSlides(HardwareMap hardwareMap) {
        slideMotor = hardwareMap.get(DcMotorEx.class, "vSlides");
        encoderSlave = hardwareMap.get(DcMotorEx.class, "rightBack");

        pidControl = new SimplePIDControl(Constants.Outtake.p, Constants.Outtake.i, Constants.Outtake.d);

        pidControl.setTargetValue(0);

        slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoderSlave.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        encoderSlave.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void manualPos(double power) {
        int temp = (int)(power * Constants.Intake.manualSlideSpeed);
        if (temp == 0) {
            lastSlidePos += temp;
            pidControl.setTargetValue(lastSlidePos);
        }
    }

    public void setPos(int newPos) {
        if (lastSlidePos != newPos) {
            lastSlidePos  = newPos;
            pidControl.setTargetValue(lastSlidePos);
        }
    }
    public void resetPower(double power) {
        slideMotor.setPower(power);
    }

    public int getTargetPos() {
        return lastSlidePos;
    }
    public int getPos() {
        return -encoderSlave.getCurrentPosition();
    }

    public void update() {
        slideMotor.setPower(pidControl.update(-encoderSlave.getCurrentPosition(), Constants.Outtake.f));
    }

}
