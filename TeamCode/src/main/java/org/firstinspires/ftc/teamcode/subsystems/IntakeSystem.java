package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.utility.ColorEnum;
import org.firstinspires.ftc.teamcode.utility.ColorFunctions;
import org.firstinspires.ftc.teamcode.utility.RobotSideEnum;

public class IntakeSystem {
    ElapsedTime time = new ElapsedTime();

    // careful about this in auto
    boolean jammed = false;

    boolean intakeOnce = false;
    double intakeTime = 2000000000;

    public IntakeClaw intakeClaw;
    private HorizontalSlides hSlides;
    private RevColorSensorV3 intakeSensor;

    private RobotSideEnum robotSide;

    public boolean unjamP2 = false;

    public IntakeSystem(HardwareMap hardwareMap, RobotSideEnum robotSide) {
        intakeClaw = new IntakeClaw(hardwareMap);
        hSlides = new HorizontalSlides(hardwareMap);

        intakeSensor = hardwareMap.get(RevColorSensorV3.class, "intakeSensor");

        this.robotSide = robotSide;
    }

    public void setIntakePower(double newIntakePower) {
        if (!jammed) {
            intakeClaw.setIntakePower(newIntakePower);
        }
    }

    public void setIntakeServoPos(double newPos) {
        intakeClaw.setIntakeServoPos(newPos);
    }
    public void setDepoServoPos(double newPos) {
        intakeClaw.setDepoServoPos(newPos);
    }

    public double getIntakeServoPos() {
        return intakeClaw.getIntakeServoPos();
    }
    public double getDepoServoPos() {
        return intakeClaw.getDepoServoPos();
    }

    public boolean isJammed() {
        return intakeClaw.isJammed();
    }

    public void manualHSlide(double power) {
        hSlides.manualPos(power);
    }

    public void setHSlidesInches(double inchesY) {
        hSlides.setPos((int)(inchesY * 1532.0 / 19.0));
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

    public boolean HSlidePressed() {
        return hSlides.tuchyWuchy();
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
        setDepoServoPos(Constants.Intake.depoStore);
        setIntakePower(0);
    }

    public void storeOutPos() {
        setHSlidePos(Constants.Intake.transferSlides);
        setIntakeServoPos(Constants.Intake.wristClear);
        setDepoServoPos(Constants.Intake.depoStore);
        setIntakePower(0);
    }

    //Artifact
    public void intakeBitMore() {
        intakeClaw.bitMore();
    }

    public boolean intakeUntilColor() {
        NormalizedRGBA currentColor = intakeSensor.getNormalizedColors();
        if (!jammed) {
            setIntakePower(Constants.Intake.intakeSpeed);
            if (robotSide == RobotSideEnum.Blue) {
                if (ColorFunctions.toColor(currentColor) == ColorEnum.blue) {
                    setIntakePower(0);
                    return true;
                } else {
                    return false;
                }
            } else if (robotSide == RobotSideEnum.Red) {
                if (ColorFunctions.toColor(currentColor) == ColorEnum.red) {
                    setIntakePower(0);
                    return true;
                } else {
                    return false;
                }
            } else {
                if (ColorFunctions.toColor(currentColor) == ColorEnum.red ) {
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
        NormalizedRGBA currentColor = intakeSensor.getNormalizedColors();
        if (!jammed) {
            setIntakePower(Constants.Intake.intakeSpeed);
            if (robotSide == RobotSideEnum.Blue) {
                if (ColorFunctions.toColor(currentColor) == ColorEnum.blue || ColorFunctions.toColor(currentColor) == ColorEnum.yellow) {
                    setIntakePower(0);
                    return true;
                } else {
                    return false;
                }
            } else if (robotSide == RobotSideEnum.Red) {
                if (ColorFunctions.toColor(currentColor) == ColorEnum.red || ColorFunctions.toColor(currentColor) == ColorEnum.yellow) {
                    setIntakePower(0);
                    return true;
                } else {
                    return false;
                }
            } else {
                if (ColorFunctions.toColor(currentColor) == ColorEnum.yellow || ColorFunctions.toColor(currentColor) == ColorEnum.red || ColorFunctions.toColor(currentColor) == ColorEnum.blue) {
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

    public boolean intakeUntilYellow() {
        NormalizedRGBA currentColor = intakeSensor.getNormalizedColors();
        if (!jammed) {
            setIntakePower(Constants.Intake.intakeSpeed);
            if (ColorFunctions.toColor(currentColor) == ColorEnum.yellow) {
                setIntakePower(0);
                return true;
            } else {
                return false;
            }
        } else {
            return false;
        }
    }

    public boolean intakeUntilColorBinary(boolean red) {
        NormalizedRGBA currentColor = intakeSensor.getNormalizedColors();
        if (!jammed) {
            setIntakePower(Constants.Intake.intakeSpeed);
            if (red) {
                if (ColorFunctions.toColor(currentColor) == ColorEnum.red) {
                    setIntakePower(0);
                    return true;
                } else {
                    return false;
                }
            } else {
                if (ColorFunctions.toColor(currentColor) == ColorEnum.blue) {
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

    //Artifact
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

    public boolean readyToTransfer() {
        return Math.abs(getHSlidePos() - Constants.Intake.transferSlides) < 20 && intakeSensor.getDistance(DistanceUnit.INCH) > Constants.Color.hasDistance;
    }

    public boolean readyToTransfer(boolean useSensor) {
        if (useSensor) {
            return Math.abs(getHSlidePos() - Constants.Intake.transferSlides) < 10 && intakeSensor.getDistance(DistanceUnit.INCH) > Constants.Color.hasDistance;
        } else {
            return Math.abs(getHSlidePos() - Constants.Intake.transferSlides) < 10;
        }
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

    public void update() {
        jammed = isJammed();
        if (jammed) {
            intakeClaw.setIntakePower(Constants.Intake.unjamSpeed);
        }
        hSlides.update();
    }

    public void disable() {
        hSlides.disable();
    }

    public void reEnable(int slidePos) {
        hSlides.reEnable(slidePos);
    }

}


