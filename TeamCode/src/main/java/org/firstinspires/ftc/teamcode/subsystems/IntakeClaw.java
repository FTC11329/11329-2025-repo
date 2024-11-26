package org.firstinspires.ftc.teamcode.subsystems;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;


public class IntakeClaw {
    DcMotor intakeMotor;
    double lastIntakePower;

    Servo wristServo;
    double lastWristPos;

    public IntakeClaw(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        wristServo = hardwareMap.get(Servo.class, "intakeServo");

        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        wristServo.setDirection(Servo.Direction.FORWARD);
    }

    public void setIntakePower(double newIntakePower) {
        if (lastIntakePower != newIntakePower) {
            lastIntakePower  = newIntakePower;
            intakeMotor.setPower(newIntakePower);
        }
    }

    public void setIntakeServoPos(double newPos) {
        if (lastWristPos != newPos) {
            lastWristPos  = newPos;
            wristServo.setPosition(newPos);
        }
    }

    public void stopIntake() {
        intakeMotor.setPower(0);
    }
}