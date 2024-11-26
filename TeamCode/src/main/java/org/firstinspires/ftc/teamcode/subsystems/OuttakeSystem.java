package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.utility.PlacePosEnum;

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
}
