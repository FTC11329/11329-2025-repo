package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;
import androidx.core.app.NotificationCompat;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.utility.PlacePosEnum;
import org.jetbrains.annotations.NotNull;

public class OuttakeSystem {
    OuttakeArm outtakeArm;
    VerticalSlides vSlides;

    public OuttakeSystem(HardwareMap hardwareMap) {
        outtakeArm = new OuttakeArm(hardwareMap);
        vSlides = new VerticalSlides(hardwareMap);
    }

    public void setArmPos(double newPos) {
        outtakeArm.setArmPos(newPos);
    }
    public void setClawPos(double newPos) {
        outtakeArm.setClawPos(newPos);
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

    public void storePos() {
        setClawPos(Constants.Outtake.dropClaw);
        setVSlidePos(Constants.Outtake.intakeSlides);
        setArmPos(Constants.Outtake.intakeArm);
    }
    public void placePos(PlacePosEnum posEnum) {
        setClawPos(Constants.Outtake.grabClaw);
        if (posEnum == PlacePosEnum.lowSpecimen) {
            setArmPos(Constants.Outtake.specimenArm);
            setVSlidePos(Constants.Outtake.lowSpecimenSlides);
        } else if (posEnum == PlacePosEnum.highSpecimen) {
            setArmPos(Constants.Outtake.specimenArm);
            setVSlidePos(Constants.Outtake.highSpecimenSlides);
        } else if (posEnum == PlacePosEnum.lowBasket) {
            setArmPos(Constants.Outtake.basketArm);
            setVSlidePos(Constants.Outtake.lowBasketSlides);
        } else if (posEnum == PlacePosEnum.highBasket) {
            setArmPos(Constants.Outtake.basketArm);
            setVSlidePos(Constants.Outtake.highBasketSlides);
        }
    }
    //Actions***************************************************************************************
    public class ToSpecimen implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            placePos(PlacePosEnum.highSpecimen);
            return false;
        }
    }
    public class PastSpecimen implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            setVSlidePos(Constants.Outtake.highSpecimenSlides + 50);
            setArmPos(Constants.Outtake.specimenArmEnd);
            return false;
        }
    }
    public class Drop implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            setClawPos(Constants.Outtake.dropClaw);
            return false;
        }
    }
    public class ToWallSpecimen implements Action {
        @Override
        public boolean run(@NotNull TelemetryPacket packet) {
            setArmPos(Constants.Outtake.intakeWallArm);
            setVSlidePos(Constants.Outtake.intakeWallSlides);
            return false;
        }
    }
    public Action toSpecimen(){
        return new ToSpecimen();
    }
    public Action pastSpecimen(){
        return new PastSpecimen();
    }
    public Action drop(){
        return new Drop();
    }
    public Action toWallSpecimen() {
        return new ToWallSpecimen();
    }
}
