package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.utility.PlacePosEnum;
import org.firstinspires.ftc.teamcode.utility.RobotSideEnum;

public class OuttakeSystem {

    public BackFlaps backFlaps;
    public OuttakeArm outtakeArm;
    public VerticalSlides vSlides;

    public OuttakeSystem(HardwareMap hardwareMap, RobotSideEnum robotSide, boolean initArm) {
        backFlaps = new BackFlaps(hardwareMap);
        vSlides = new VerticalSlides(hardwareMap);
        outtakeArm = new OuttakeArm(hardwareMap, robotSide, initArm);
    }

    public void initArm() {
        outtakeArm.initArm();
    }

    public void manualArm(double power) {
        outtakeArm.manualArmPos(power);
    }

    public void setArmPos(double newPos) {
        outtakeArm.setArmPos(newPos);
    }

    public void setFlapsUp() {
        backFlaps.setFlapsUp();
    }
    public void setFlapsWall() {
        backFlaps.setFlapsWall();
    }
    public void setFlapsSpikeClear() {
        backFlaps.setFlapsSpikeClear();
    }

    public void setBackFlaps(double set) {
        backFlaps.setBackFlaps(set);
    }

    public double getBackFlapsPos() {
        return backFlaps.getPos();
    }

    public void manualWrist(double power) {
        outtakeArm.manualWristPos(power);
    }

    public void setWristPos(double newPos) {
        outtakeArm.setWristPos(newPos);
    }

    public double getArmPos() {
        return outtakeArm.getArmPos();
    }

    public double getWristPos() {
        return outtakeArm.getWristPos();
    }

    public void setClawPos(double newPos) {
        outtakeArm.setClawPos(newPos);
    }

    public void manualVSlide(double power) {
        vSlides.manualPos(power);
    }

    public void setVSlidePos(int newPos) {
        vSlides.setPos(newPos);
    }

    public int getVSlideTargetPos() {
        return vSlides.getTargetPos();
    }

    public int getVSlidePos() {
        return vSlides.getPos();
    }

    public double getClawDistance() {
        return outtakeArm.getSensorDistance();
    }

    public void storePos() {
        setClawPos(Constants.Outtake.dropClaw);
        setVSlidePos(Constants.Outtake.intakeSlides);
        setArmPos(Constants.Outtake.intakeArm);
    }

    public void placePos(PlacePosEnum posEnum) {
        if (posEnum == PlacePosEnum.preClipHighSpecimenAuto) {
            setArmPos(Constants.Outtake.highSpecimenArmAuto);
            setVSlidePos(Constants.Outtake.highSpecimenSlidesAutoPre);
            setClawPos(Constants.Outtake.grabClaw);
            setWristPos(Constants.Outtake.highSpecimenWristAutoPre);

        } else if (posEnum == PlacePosEnum.preClipLowSpecimenAuto) {
            setArmPos(Constants.Outtake.lowSpecimenArmAutoPre);
            setVSlidePos(Constants.Outtake.lowSpecimenSlidesAutoPre);
            setClawPos(Constants.Outtake.grabClaw);
            setWristPos(Constants.Outtake.lowSpecimenWristAuto);

        } else if (posEnum == PlacePosEnum.postClipHighSpecimenAuto) {
            setVSlidePos(Constants.Outtake.highSpecimenSlidesAutoPost);
            setWristPos(Constants.Outtake.highSpecimenWristAutoPost);

        } else if (posEnum == PlacePosEnum.postClipLowSpecimenAuto) {
            setWristPos(Constants.Outtake.lowSpecimenWristAutoPost);
//            setArmPos(Constants.Outtake.lowSpecimenArmAutoPost);
//            setVSlidePos(Constants.Outtake.lowSpecimenSlidesAutoPost);

        } else if (posEnum == PlacePosEnum.postClipHighSpecimen) {
//            setArmPos(Constants.Outtake.postHighSpecimenArm);
//            setVSlidePos(Constants.Outtake.postClipHighSpecimenSlides);
            setWristPos(Constants.Outtake.postClipHighSpecimenWrist);
            setClawPos(Constants.Outtake.grabClaw);

        } else if (posEnum == PlacePosEnum.postClipLowSpecimen) {
            setArmPos(Constants.Outtake.postLowSpecimenArm);
            setVSlidePos(Constants.Outtake.postClipLowSpecimenSlides);
            setClawPos(Constants.Outtake.grabClaw);
            setWristPos(Constants.Outtake.lowSpecimenWristAutoPost);

        } else if (posEnum == PlacePosEnum.safeHighSpecimen) {
            setArmPos(Constants.Outtake.safeHighSpecimenArm);
            setVSlidePos(Constants.Outtake.safeHighSpecimenSlides);
            setClawPos(Constants.Outtake.grabClaw);
            setWristPos(Constants.Outtake.safeHighSpecimenWrist);

        } else if (posEnum == PlacePosEnum.safeLowSpecimen) {
            setArmPos(Constants.Outtake.safeLowSpecimenArm);
            setVSlidePos(Constants.Outtake.safeLowSpecimenSlides);
            setClawPos(Constants.Outtake.grabClaw);
            setWristPos(Constants.Outtake.safeLowSpecimenWrist);

        } else if (posEnum == PlacePosEnum.lowSpecimen) {
            setArmPos(Constants.Outtake.lowSpecimenArm);
            setVSlidePos(Constants.Outtake.lowSpecimenSlides);
            setClawPos(Constants.Outtake.grabClaw);
            setWristPos(Constants.Outtake.lowSpecimenWristAuto);

        } else if (posEnum == PlacePosEnum.highSpecimen) {
            setArmPos(Constants.Outtake.highSpecimenArm);
            setVSlidePos(Constants.Outtake.highSpecimenSlides);
            setClawPos(Constants.Outtake.grabClaw);
            setWristPos(Constants.Outtake.preClipHighSpecimenWrist);

        } else if (posEnum == PlacePosEnum.lowBasket) {
            setArmPos(Constants.Outtake.basketArm);
            setVSlidePos(Constants.Outtake.lowBasketSlides);
            setClawPos(Constants.Outtake.grabClaw);
            setWristPos(Constants.Outtake.basketWrist);

        } else if (posEnum == PlacePosEnum.highBasket) {
            setArmPos(Constants.Outtake.basketArm);
            setVSlidePos(Constants.Outtake.highBasketSlides);
            setClawPos(Constants.Outtake.grabClaw);
            setWristPos(Constants.Outtake.basketWrist);

        } else if (posEnum == PlacePosEnum.wall) {
            setArmPos(Constants.Outtake.intakeWallArm);
            setVSlidePos(Constants.Outtake.intakeWallSlides);
            setWristPos(Constants.Outtake.wallWrist);

        } else if (posEnum == PlacePosEnum.wallAuto) {
            setArmPos(Constants.Outtake.intakeWallArm);
            setVSlidePos(Constants.Outtake.intakeWallAutoSlides);
            setWristPos(Constants.Outtake.wallWrist);

        } else if (posEnum == PlacePosEnum.intake) {
            setArmPos(Constants.Outtake.intakeArm);
            setVSlidePos(Constants.Outtake.intakeSlides);
            setWristPos(Constants.Outtake.intakeWrist);
        }
    }

    public double getAmp() {
        return vSlides.getAmp();
    }

    public boolean overAmp() {
        return vSlides.overAmp();
    }

    public boolean readyToTransfer() {
        return readyToTransfer(false);
    }
    public boolean readyToTransfer(boolean useSensor) {
        if (useSensor) {
            return Math.abs(getVSlidePos() - Constants.Outtake.intakeSlides) < 20 && Math.abs(getArmPos() - Constants.Outtake.intakeArm) < 0.1 && outtakeArm.seesTransfer();
        } else {
            return Math.abs(getVSlidePos() - Constants.Outtake.intakeSlides) < 20 && Math.abs(getArmPos() - Constants.Outtake.intakeArm) < 0.1;
        }
    }
    public boolean seesWall() {
        return outtakeArm.seesWall();
    }
    public boolean seesTransfer() {
        return outtakeArm.seesWall();
    }

    public void update(boolean reZeroButton) {
        vSlides.update(reZeroButton);
    }

    public void disable() {
        vSlides.disable();
    }
    public void reEnable(int slidePos) {
        vSlides.reEnable(slidePos);
    }
}