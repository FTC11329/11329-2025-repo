package org.firstinspires.ftc.teamcode.criAutos;

import org.firstinspires.ftc.teamcode.pedropathing.localization.Pose;

public class CommonPoses {
    // I have decided that 0 heading will be the way the drive team is facing
    // Meaning also +x is the same direction
    // Meaning +y is left and -y is right

    public static Pose startLeftOuter  = new Pose(-64, 86.5, Math.toRadians(-90));
    public static Pose startLeftInner  = new Pose(-64, 31.5, Math.toRadians(90));
    public static Pose startRightInner = new Pose(-64, -31.5, Math.toRadians(-90));
    public static Pose startRightOuter = new Pose(-64, -86.5, Math.toRadians(90));


    public static Pose redBasket  = new Pose(55, 99.5, Math.toRadians(-45));
    public static Pose blueBasket = new Pose(55, -99.5, Math.toRadians(45));

    public static Pose intakeSubLeftOuter   = new Pose();
    public static Pose intakeSubLeftMiddle  = new Pose();
    public static Pose intakeSubRightMiddle = new Pose();
    public static Pose intakeSubRightOuter  = new Pose();

    public static Pose pickupWallLeft  = new Pose(-59.5,  12, Math.toRadians(0));
    public static Pose pickupWallRight = new Pose(-58.5, -13.5, Math.toRadians(0));



    public static Pose barLeftOuterTop  = new Pose(5.5, 79.5, Math.toRadians(-90));
    public static Pose barLeftOuterMid  = new Pose(0, 79.5, Math.toRadians(-90));
    public static Pose barLeftOuterBot  = new Pose(-5.5, 79.5, Math.toRadians(-90));

    public static Pose barLeftInnerTop  = new Pose(5,  16.1, Math.toRadians(90));
    public static Pose barLeftInnerMid  = new Pose(0,    16.1, Math.toRadians(90));
    public static Pose barLeftInnerBot  = new Pose(-5, 16.1, Math.toRadians(90));

    public static Pose barRightInnerTop = new Pose(5.5,  -18, Math.toRadians(-90));
    public static Pose barRightInnerMid = new Pose(0,    -18, Math.toRadians(-90));
    public static Pose barRightInnerBot = new Pose(-5.5, -18, Math.toRadians(-90));

    public static Pose barRightOuterTop = new Pose(5.5,  -79.5, Math.toRadians(90));
    public static Pose barRightOuterMid = new Pose(0,    -79.5, Math.toRadians(90));
    public static Pose barRightOuterBot = new Pose(-5.5, -79.5, Math.toRadians(90));

    public static Pose innerSpikeRightTop = new Pose(-2,  -14, Math.toRadians(-90));
    public static Pose innerSpikeRightMid = new Pose(-8, -14, Math.toRadians(-90));
    public static Pose innerSpikeRightBot = new Pose(-21, -14, Math.toRadians(-90));

    public static Pose innerSpikeLeftTop  = new Pose(-2,  12.7, Math.toRadians(90));
    public static Pose innerSpikeLeftMid  = new Pose(-8, 12.7, Math.toRadians(90));
    public static Pose innerSpikeLeftBot  = new Pose(-21, 12.7, Math.toRadians(90));

    // 1,2,3: inner, middle, outer
    public static Pose leftOuterSpike3 = new Pose(-49, 82, Math.toRadians(90));
    public static Pose leftOuterSpike2 = new Pose(-51.2, 81.9, Math.toRadians(90));
    public static Pose leftOuterSpike1 = new Pose(-51, 76, Math.toRadians(90));

    public static Pose rightOuterSpike1 = new Pose(-49, -82, Math.toRadians(-90));
    public static Pose rightOuterSpike2 = new Pose(-51.2, -81.9, Math.toRadians(-90));
    public static Pose rightOuterSpike3 = new Pose(-51, -76, Math.toRadians(-90));

}
