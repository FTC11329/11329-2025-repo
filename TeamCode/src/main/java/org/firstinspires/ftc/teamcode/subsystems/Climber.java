package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Climber {

    DcMotorEx climberMotor;
    int lastClimberPos;

    public Climber(HardwareMap hardwareMap) {
        climberMotor = hardwareMap.get(DcMotorEx.class, "climber");

        climberMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        climberMotor.setTargetPosition(0);
        climberMotor.setPower(1);
        climberMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        climberMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        climberMotor.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public void setPos(int newPos) {
        if (lastClimberPos != newPos) {
            lastClimberPos = newPos;
            climberMotor.setTargetPosition(newPos);
        }
    }
    public void setPower(double pow) {
        climberMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        climberMotor.setPower(pow);
    }
    public int getTargetPos() {
        return lastClimberPos;
    }
    public int getPos() {
        return climberMotor.getCurrentPosition();
    }



}
