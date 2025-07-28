package org.firstinspires.ftc.teamcode.criAutos;

import org.firstinspires.ftc.teamcode.pedropathing.localization.Pose;

public class CommonPoses {
    // I have decided that 0 heading will be the way the drive team is facing
    // Meaning also +x is the same direction
    // Meaning +y is left and -y is right

    public static Pose startLeftOuter  = new Pose(-64, 84.5, Math.toRadians(-90));
    public static Pose startLeftInner  = new Pose(-64, 33.5, Math.toRadians(90));
    public static Pose startRightInner = new Pose(-64, -33.5, Math.toRadians(-90));
    public static Pose startRightOuter = new Pose(-64, -84.5, Math.toRadians(90));


    public static Pose redBasket  = new Pose(-52.5, 97.5, Math.toRadians(-45));
    public static Pose blueBasket = new Pose(-55, -100, Math.toRadians(45));

    public static Pose intakeSubLeftOuter   = new Pose(-24, 56);
    public static Pose intakeSubLeftMiddle  = new Pose(-24, 53);
    public static Pose intakeSubRightMiddle = new Pose(-24, -53);
    public static Pose intakeSubRightOuter  = new Pose(-24, -56);

    public static Pose pickupWallLeft  = new Pose(-61,  13.5, Math.toRadians(0));
    public static Pose pickupWallRight = new Pose(-61, -13.5, Math.toRadians(0));



    public static Pose barLeftOuterTop  = new Pose(6, 78.5, Math.toRadians(-90));
    public static Pose barLeftOuterMid  = new Pose(0, 78.5, Math.toRadians(-90));
    public static Pose barLeftOuterBot  = new Pose(-5.5, 78.5, Math.toRadians(-90));

    public static Pose barLeftInnerTop  = new Pose(-3,  15.5, Math.toRadians(90));
    public static Pose barLeftInnerMid  = new Pose(-3,15.5, Math.toRadians(90));
    public static Pose barLeftInnerBot  = new Pose(-5, 15.5, Math.toRadians(90));

    public static Pose barRightInnerTop = new Pose(-3,  -18, Math.toRadians(-90));
    public static Pose barRightInnerMid = new Pose(-3,-18, Math.toRadians(-90));
    public static Pose barRightInnerBot = new Pose(-5, -18, Math.toRadians(-90));

    public static Pose barRightOuterTop = new Pose(5.5,  -79.5, Math.toRadians(90));
    public static Pose barRightOuterMid = new Pose(0,    -79.5, Math.toRadians(90));
    public static Pose barRightOuterBot = new Pose(-5.5, -79.5, Math.toRadians(90));

    public static Pose innerSpikeRightTop = new Pose(0,  -14, Math.toRadians(-90)); // not perfect
    public static Pose innerSpikeRightMid = new Pose(-18, -14, Math.toRadians(-90));
    public static Pose innerSpikeRightBot = new Pose(-27, -14, Math.toRadians(-90));

    public static Pose innerSpikeLeftTop  = new Pose(0,  12.7, Math.toRadians(90)); // not perfect
    public static Pose innerSpikeLeftMid  = new Pose(-18, 12.7, Math.toRadians(90));
    public static Pose innerSpikeLeftBot  = new Pose(-27, 12.7, Math.toRadians(90));

    // 1,2,3: inner, middle, outer
    public static Pose leftOuterSpike3 = new Pose(-49, 106, Math.toRadians(27));
    public static Pose leftOuterSpike2 = new Pose(-51.2, 105.9, Math.toRadians(0));
    public static Pose leftOuterSpike1 = new Pose(-51, 100, Math.toRadians(-9));

    public static Pose rightOuterSpike1 = new Pose(-49, -106, Math.toRadians(27));
    public static Pose rightOuterSpike2 = new Pose(-51.2, -105.9, Math.toRadians(0));
    public static Pose rightOuterSpike3 = new Pose(-51, -100, Math.toRadians(-9));

}
