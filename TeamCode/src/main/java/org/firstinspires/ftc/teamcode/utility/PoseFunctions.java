package org.firstinspires.ftc.teamcode.utility;

import com.pedropathing.localization.Pose;

public class PoseFunctions {
    private static final double sideSubY = -24;

    private static final double basketX = -24;
    private static final double observationX = 24;

    public static LocationEnum getLocation(Pose currentPose) {
        if (currentPose.getY() > -sideSubY) {
            return LocationEnum.otherSide;
        } else if (currentPose.getY() > sideSubY) {
            if (currentPose.getX() > 0) {
                return LocationEnum.rightSideSub;
            } else {
                return LocationEnum.leftSideSub;
            }
        } else {
            if (currentPose.getX() < basketX) {
                return LocationEnum.basket;
            } else if (currentPose.getX() < observationX) {
                return LocationEnum.frontSub;
            } else {
                return LocationEnum.observation;
            }
        }
    }
}
