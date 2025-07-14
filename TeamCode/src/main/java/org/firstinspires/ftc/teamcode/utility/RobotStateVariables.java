package org.firstinspires.ftc.teamcode.utility;

public class RobotStateVariables {
    public RobotSideEnum robotSide;

    public PlacePosEnum whereAmI;
    public boolean hasInIntake;
    public boolean hasInOutake;

    public boolean clawToggle = true;
    public RobotStateVariables(PlacePosEnum whereAmI, RobotSideEnum robotSide) {
        this(whereAmI, false, false, robotSide);
    }
    public RobotStateVariables(PlacePosEnum whereAmI, boolean hasInIntake, boolean hasInOutake, RobotSideEnum robotSide) {
        this.whereAmI = whereAmI;
        this.hasInIntake = hasInIntake;
        this.hasInOutake = hasInOutake;
        this.robotSide = robotSide;
    }
}
