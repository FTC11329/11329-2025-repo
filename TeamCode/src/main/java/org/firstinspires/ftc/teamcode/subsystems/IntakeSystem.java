package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.utility.ColorEnum;
import org.firstinspires.ftc.teamcode.utility.ColorFunctions;
import org.firstinspires.ftc.teamcode.utility.RobotSideEnum;

public class IntakeSystem {

    IntakeClaw intakeClaw;
    HorizontalSlides hSlides;
    RevColorSensorV3 clawSensor;

    RobotSideEnum robotSide;

    public IntakeSystem(HardwareMap hardwareMap, RobotSideEnum robotSide) {
        intakeClaw = new IntakeClaw(hardwareMap);
        hSlides = new HorizontalSlides(hardwareMap);

        clawSensor = hardwareMap.get(RevColorSensorV3.class, "clawSensor");

        this.robotSide = robotSide;
    }

    public void setIntakePower(double newIntakePower) {
        intakeClaw.setIntakePower(newIntakePower);
    }
    public void setIntakeServoPos(double newPos) {
        intakeClaw.setIntakeServoPos(newPos);
    }


    public void setHSlidePos(int newPos) {
        hSlides.setPos(newPos);
    }
    public int getHSlideTargetPos() {
        return hSlides.getTargetPos();
    }
    public int getHSlidePos() {
        return hSlides.getPos();
    }


    public void pickupPos(int hSlidePos) {
        setHSlidePos(hSlidePos);
        setIntakeServoPos(Constants.Intake.wristDown);
    }

    public void storePos() {
        setHSlidePos(Constants.Intake.minSlidePos);
        setIntakeServoPos(Constants.Intake.wristUp);
        setIntakePower(0);
    }


    public boolean intakeUntilColor() {
        setIntakePower(Constants.Intake.intakeSpeed);
        if (clawSensor.getDistance(DistanceUnit.INCH) < Constants.Color.hasDistance) {
            if (robotSide == RobotSideEnum.Blue) {
                if (ColorFunctions.toColor(clawSensor.getNormalizedColors()) == ColorEnum.blue) {
                    setIntakePower(0);
                    return true;
                } else {
                    return false;
                }
            } else {
                if (ColorFunctions.toColor(clawSensor.getNormalizedColors()) == ColorEnum.red) {
                    setIntakePower(0);
                    return true;
                } else {
                    return false;
                }
            }
        } else {
            return false;
        }
    }

    public boolean intakeUntil() {
        setIntakePower(Constants.Intake.intakeSpeed);
        if (clawSensor.getDistance(DistanceUnit.INCH) < Constants.Color.hasDistance) {
            if (robotSide == RobotSideEnum.Blue) {
                if (ColorFunctions.toColor(clawSensor.getNormalizedColors()) == ColorEnum.blue || ColorFunctions.toColor(clawSensor.getNormalizedColors()) == ColorEnum.yellow) {
                    setIntakePower(0);
                    return true;
                } else {
                    return false;
                }
            } else {
                if (ColorFunctions.toColor(clawSensor.getNormalizedColors()) == ColorEnum.red || ColorFunctions.toColor(clawSensor.getNormalizedColors()) == ColorEnum.yellow) {
                    setIntakePower(0);
                    return true;
                } else {
                    return false;
                }
            }
        } else {
            return false;
        }

    }
    public NormalizedRGBA directColor() {
        return clawSensor.getNormalizedColors();
    }
    public ColorEnum color() {
        return ColorFunctions.toColor(clawSensor.getNormalizedColors());
    }

    public double distance() {
        return clawSensor.getDistance(DistanceUnit.INCH);
    }

    public void stop() {
        intakeClaw.stopIntake();
    }
}
