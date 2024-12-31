package org.firstinspires.ftc.teamcode.subsystems;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Constants;


public class IntakeClaw {
    DcMotor intakeMotor;
    double lastIntakePower = 0;

    Servo intakeServo;
    double lastWristPos = 0;

    public IntakeClaw(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        intakeServo = hardwareMap.get(Servo.class, "intakeServo");

        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intakeServo.setDirection(Servo.Direction.REVERSE);

        intakeServo.setPosition(0);
    }

    public void setIntakePower(double newIntakePower) {
        if (lastIntakePower != newIntakePower) {
            intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            lastIntakePower  = newIntakePower;
            intakeMotor.setPower(newIntakePower);
        }
    }

    public void bitMore() {
        intakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeMotor.setTargetPosition(0);
        intakeMotor.setPower(1);
        intakeMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        intakeMotor.setTargetPosition(Constants.Intake.bitMore);
    }


    public void setIntakeServoPos(double newPos) {
        if (lastWristPos != newPos) {
            lastWristPos  = newPos;
            intakeServo.setPosition(newPos);
        }
    }
    public double getIntakeServoPos() {
        return intakeServo.getPosition();
    }

    public void stopIntake() {
        intakeMotor.setPower(0);
    }
}