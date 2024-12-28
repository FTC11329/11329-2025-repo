package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.utility.PlacePosEnum;
import org.jetbrains.annotations.NotNull;

public class OuttakeSystem {
    ElapsedTime time = new ElapsedTime();

    public OuttakeArm outtakeArm;
    VerticalSlides vSlides;

    public OuttakeSystem(HardwareMap hardwareMap) {
        outtakeArm = new OuttakeArm(hardwareMap);
        vSlides = new VerticalSlides(hardwareMap);
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
            setClawPos(Constants.Outtake.dropClaw);
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
        boolean init = false;

        double slideTime = 2000000000;
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!init) {
                slideTime = time.milliseconds();
                init = true;
            }
            if (time.milliseconds() > slideTime + 200) {
                return false;
            }
            return true;
        }
    }
    public class Drop implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            setClawPos(Constants.Outtake.dropClaw);
            return false;
        }
    }
    public class Grab implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            setClawPos(Constants.Outtake.grabClaw);
            return false;
        }
    }
    public class AboveWall implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            setVSlidePos(Constants.Outtake.safeFromWallSlides);
            return false;
        }
    }

    public class ToWallSpecimen implements Action {
        boolean initialized = false;
        double startTime = 0;

        @Override
        public boolean run(@NotNull TelemetryPacket packet) {
            if (!initialized) {
                startTime = time.milliseconds();
                initialized = true;
            }
            if (time.milliseconds() < startTime + 50) {
                setClawPos(Constants.Outtake.grabClaw);
                setVSlidePos(Constants.Outtake.intakeWallSlides);
            }
            if (time.milliseconds() > startTime + 200 && time.milliseconds() < startTime + 250) {
                setArmPos(Constants.Outtake.intakeWallArm);
            }
            if (time.milliseconds() > startTime + 400 && time.milliseconds() < startTime + 450) {
                setClawPos(Constants.Outtake.dropClaw);
                return false;
            }
            return true;
        }
    }
    public class EndAutoAction implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            setArmPos(Constants.Outtake.initTeleopArm);
            setVSlidePos(0);
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
    public Action grab(){
        return new Grab();
    }
    public Action aboveWall(){
        return new AboveWall();
    }
    public Action toWallSpecimen() {
        return new ToWallSpecimen();
    }
    public Action endAutoAction() {
        return new EndAutoAction();
    }
}
