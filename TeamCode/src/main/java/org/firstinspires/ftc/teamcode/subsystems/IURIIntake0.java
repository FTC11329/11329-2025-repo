package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class IURIIntake0 {
    DcMotorEx armMotor;
    int lastClimberPos = 0;

    CRServo roller;
    double lastPower = 0;
    public IURIIntake0(HardwareMap hardwareMap) {
        roller = hardwareMap.get(CRServo.class, "roller");

        roller.setDirection(CRServo.Direction.FORWARD);

        armMotor = hardwareMap.get(DcMotorEx.class, "climber");

        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setTargetPosition(0);
        armMotor.setPower(1);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public void setPos(int newPos) {
        if (lastClimberPos != newPos) {
            lastClimberPos = newPos;
            armMotor.setTargetPosition(newPos);
        }
    }

    public void setRollerPower(double newPower) {
        if (lastPower != newPower) {
            lastPower = newPower;
            roller.setPower(newPower);
        }
    }

    public void intake() {
        setPos(140);
        setRollerPower(1);
    }

    public void store() {
        setPos(0);
        setRollerPower(0);
    }

    public void spit() {
        setRollerPower(-1);
    }
}
