package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.roadrunner.FailoverAction;
import org.firstinspires.ftc.teamcode.utility.ColorEnum;
import org.firstinspires.ftc.teamcode.utility.ColorFunctions;
import org.firstinspires.ftc.teamcode.utility.RobotSideEnum;

public class IntakeSystem {
    ElapsedTime time = new ElapsedTime();

    boolean intakeOnce = false;
    double intakeTime = 2000000000;

    public IntakeClaw intakeClaw;
    private HorizontalSlides hSlides;
    private RevColorSensorV3 intakeSensor;

    private RobotSideEnum robotSide;

    public boolean unjamP0 = false;
    public boolean unjamP2 = false;

    public IntakeSystem(HardwareMap hardwareMap, RobotSideEnum robotSide) {
        intakeClaw = new IntakeClaw(hardwareMap);
        hSlides = new HorizontalSlides(hardwareMap);

        intakeSensor = hardwareMap.get(RevColorSensorV3.class, "intakeSensor");

        this.robotSide = robotSide;
    }

    public void setIntakePower(double newIntakePower) {
        intakeClaw.setIntakePower(newIntakePower);
    }
    public void setIntakeServoPos(double newPos) {
        intakeClaw.setIntakeServoPos(newPos);
    }


    public void manualHSlide(double power) {
        hSlides.manualPos(power);
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


    public void pickupPosWithTime(int hSlidePos) {
        if (!intakeOnce) {
            intakeTime = time.milliseconds();
            intakeOnce = true;
        }
        setHSlidePos(hSlidePos);
        if (time.milliseconds() > intakeTime + 250 && time.milliseconds() < intakeTime + 300) {
            setIntakeServoPos(Constants.Intake.wristDown);
            intakeOnce = false;
            intakeTime = 2000000000;
        }
    }
    public void pickupPosWithTime() {
        pickupPosWithTime(Constants.Intake.intakeSlidePos);
    }

    public void pickupPos(int hSlidePos) {
        setHSlidePos(hSlidePos);
        setIntakeServoPos(Constants.Intake.wristDown);
    }
    public void pickupPos() {
        pickupPos(Constants.Intake.intakeSlidePos);
    }

    public void storePos() {
        setHSlidePos(Constants.Intake.minSlidePos);
        setIntakeServoPos(Constants.Intake.wristStore);
        setIntakePower(0);
    }
    public void storeOutPos() {
        setHSlidePos(Constants.Intake.minWhileDownPos);
        setIntakeServoPos(Constants.Intake.wristStore);
        setIntakePower(0);
    }


    public void intakeBitMore() {
        intakeClaw.bitMore();
    }

    public boolean intakeUntilColor() {
        setIntakePower(Constants.Intake.intakeSpeed);
        if (intakeSensor.getDistance(DistanceUnit.INCH) < Constants.Color.hasDistance) {
            if (robotSide == RobotSideEnum.Blue) {
                if (ColorFunctions.toColor(intakeSensor.getNormalizedColors()) == ColorEnum.blue) {
                    setIntakePower(0);
                    return true;
                } else {
                    return false;
                }
            } else {
                if (ColorFunctions.toColor(intakeSensor.getNormalizedColors()) == ColorEnum.red) {
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
        if (intakeSensor.getDistance(DistanceUnit.INCH) < Constants.Color.hasDistance) {
            if (robotSide == RobotSideEnum.Blue) {
                if (ColorFunctions.toColor(intakeSensor.getNormalizedColors()) == ColorEnum.blue || ColorFunctions.toColor(intakeSensor.getNormalizedColors()) == ColorEnum.yellow) {
                    setIntakePower(0);
                    return true;
                } else {
                    return false;
                }
            } else {
                if (ColorFunctions.toColor(intakeSensor.getNormalizedColors()) == ColorEnum.red || ColorFunctions.toColor(intakeSensor.getNormalizedColors()) == ColorEnum.yellow) {
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
    public boolean unjam() {
        setIntakePower(Constants.Intake.unjamSpeed);
        if (intakeSensor.getDistance(DistanceUnit.INCH) > Constants.Color.hasDistance && !unjamP2) {
            unjamP2 = true;
        }
        if (unjamP2) {
            if (intakeUntil()) {
                unjamP2 = false;
                setIntakePower(0);
                intakeBitMore();
                return true;
            }
        }
        return false;
    }
    public boolean readyToTranfer() {
        return getHSlidePos() < Constants.Intake.safeTransferSlide;
    }
    public NormalizedRGBA directColor() {
        return intakeSensor.getNormalizedColors();
    }
    public ColorEnum color() {
        return ColorFunctions.toColor(intakeSensor.getNormalizedColors());
    }

    public double distance() {
        return intakeSensor.getDistance(DistanceUnit.INCH);
    }

    public void stop() {
        intakeClaw.stopIntake();
    }


    //Actions***************************************************************************************


    private class Extend implements Action {
        int pos;
        public Extend(int pos) {
            this.pos = pos;
        }
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (pos == 100000) {
                pickupPos();
            } else {
                pickupPos(pos);
            }
            return false;
        }
    }

    private class Retract implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            storePos();
            return false;
        }
    }

    private class IntakeColorMoving implements Action {

        FailoverAction failoverAction;

        public IntakeColorMoving(FailoverAction failoverAction) {
            this.failoverAction = failoverAction;
        }
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            boolean hasThing = intakeUntilColor();
            if (hasThing) {
                failoverAction.failover();
            }
            return !hasThing;
        }

    }
    private class IntakeColor implements Action {
        boolean initialized = false;
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                setIntakePower(Constants.Intake.intakeSpeed);
                initialized = true;
            }
            return !intakeUntilColor();
        }
    }
    private class Spit implements Action {
        ElapsedTime time = new ElapsedTime();
        double tempTime;
        boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                tempTime = time.milliseconds() + 1000;
                setIntakePower(-0.5);
                initialized = true;
            }
            if (tempTime < time.milliseconds()) {
                setIntakePower(0);
                return false;
            }
            return true;
        }
    }

    public Action extend(int pos) {
        return new Extend(pos);
    }
    public Action extend() {
        return new Extend(100000);
    }
    public Action retract() {
        return new Retract();
    }

    public Action intakeColor(FailoverAction a) {
        return new IntakeColorMoving(a);
    }
    public Action intakeColor() {
        return new IntakeColor();
    }
    public Action spit() {
        return new Spit();
    }

}


