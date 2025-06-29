package org.firstinspires.ftc.teamcode.criAutos;

import org.firstinspires.ftc.teamcode.pedropathing.localization.Pose;

public class CommonPoses {
    // I have decided that 0 heading will be the way the drive team is facing
    // Meaning also +x is the same direction
    // Meaning +y is left and -y is right

    /*
    // todo make offset work like this
    public static Pose offset = new Pose();

    public static void setOffset(Pose offset) {
        this.offset = offset
    }
     */

    //todo:
    public static Pose startLeftOuter  = new Pose();
    public static Pose startLeftInner  = new Pose();
    public static Pose startRightInner = new Pose();
    public static Pose startRightOuter = new Pose(-63.5, -86.5, Math.toRadians(90));


    public static Pose redBasket  = new Pose();
    public static Pose blueBasket = new Pose();

    public static Pose intakeSubLeftOuter   = new Pose();
    public static Pose intakeSubLeftMiddle  = new Pose();
    public static Pose intakeSubRightMiddle = new Pose();
    public static Pose intakeSubRightOuter  = new Pose();

    public static Pose pickupWallLeft  = new Pose(59.5,  13.5, Math.toRadians(0));
    public static Pose pickupWallRight = new Pose(-59.5, -13.5, Math.toRadians(0));



    public static Pose barLeftOuterTop  = new Pose();
    public static Pose barLeftOuterMid  = new Pose();
    public static Pose barLeftOuterBot  = new Pose();

    public static Pose barLeftInnerTop  = new Pose();
    public static Pose barLeftInnerMid  = new Pose();
    public static Pose barLeftInnerBot  = new Pose();

    public static Pose barRightInnerTop = new Pose();
    public static Pose barRightInnerMid = new Pose();
    public static Pose barRightInnerBot = new Pose();

    public static Pose barRightOuterTop = new Pose(3.5 , -79, Math.toRadians(90));
    public static Pose barRightOuterMid = new Pose(0 , -79, Math.toRadians(90));
    public static Pose barRightOuterBot = new Pose(-3.5, -79, Math.toRadians(90));
}
