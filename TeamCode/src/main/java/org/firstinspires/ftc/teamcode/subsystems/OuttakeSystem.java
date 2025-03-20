package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.utility.PlacePosEnum;

public class OuttakeSystem {
    ElapsedTime time = new ElapsedTime();

    public OuttakeArm outtakeArm;
    public VerticalSlides vSlides;

    public OuttakeSystem(HardwareMap hardwareMap, boolean initArm) {
        vSlides = new VerticalSlides(hardwareMap);
        outtakeArm = new OuttakeArm(hardwareMap, initArm);
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

    public double getArmPos() {
        return outtakeArm.getArmPos();
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

    public boolean VSlidePressed() {
        return vSlides.nearlyTuchyWuchyed();
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
        if (posEnum == PlacePosEnum.highSpecimen) {
            setArmPos(Constants.Outtake.specimenArm);
            setVSlidePos(Constants.Outtake.highSpecimenSlides);
            setClawPos(Constants.Outtake.grabClaw);

        } else if (posEnum == PlacePosEnum.lowBasket) {
            setArmPos(Constants.Outtake.basketArm);
            setVSlidePos(Constants.Outtake.lowBasketSlides);
            setClawPos(Constants.Outtake.grabClaw);

        } else if (posEnum == PlacePosEnum.highBasket) {
            setArmPos(Constants.Outtake.basketArm);
            setVSlidePos(Constants.Outtake.highBasketSlides);
            setClawPos(Constants.Outtake.grabClaw);

        } else if (posEnum == PlacePosEnum.wall) {
            setArmPos(Constants.Outtake.intakeWallArm);
            setVSlidePos(Constants.Outtake.intakeWallSlides);

        } else if (posEnum == PlacePosEnum.wallAuto) {
            setArmPos(Constants.Outtake.intakeWallArm);
            setVSlidePos(Constants.Outtake.intakeWallAutoSlides);

        } else if (posEnum == PlacePosEnum.intake) {
            setArmPos(Constants.Outtake.intakeArm);
            setVSlidePos(Constants.Outtake.intakeSlides);
        }
    }

    public double getAmp() {
        return vSlides.getAmp();
    }

    public boolean overAmp() {
        return vSlides.overAmp();
    }

    public boolean readyToTransfer() {
        return Math.abs(getVSlidePos() - Constants.Outtake.intakeSlides) < 40 && Math.abs(getArmPos() - Constants.Outtake.intakeArm) < 0.1 && outtakeArm.seesTransfer();
    }
    public boolean seesWall() {
        return outtakeArm.seesWall();
    }
    public boolean seesTransfer() {
        return outtakeArm.seesWall();
    }

    public void update() {
        vSlides.update();
    }

    public void disable() {
        vSlides.disable();
    }
    public void reEnable(int slidePos) {
        vSlides.reEnable(slidePos);
    }
}