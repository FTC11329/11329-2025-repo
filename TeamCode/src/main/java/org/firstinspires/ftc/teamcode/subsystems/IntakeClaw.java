package org.firstinspires.ftc.teamcode.subsystems;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Constants;


public class IntakeClaw {
    ElapsedTime time = new ElapsedTime();

    public DcMotorEx intakeMotor;
    double lastIntakePower = 0;
    double startJamTime;
    boolean overCurrentDe = false;

    Servo intakeServo;
    double lastWristPos = 0;

    public IntakeClaw(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        intakeServo = hardwareMap.get(Servo.class, "intakeServo");

        intakeMotor.setCurrentAlert(3, CurrentUnit.AMPS);

        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intakeServo.setDirection(Servo.Direction.REVERSE);

        intakeServo.setPosition(0);
    }
    //checks if it has been jammed for more than a certain amount of time
    public boolean isJammed() {
        boolean jamBool = intakeMotor.isOverCurrent();

        if (jamBool && !overCurrentDe) {
            startJamTime = time.milliseconds();
            overCurrentDe = true;
        } else if (!jamBool) {
            overCurrentDe = false;
        }
        if (jamBool && time.milliseconds() > startJamTime + 300) {
            return true;
        }
        return false;
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