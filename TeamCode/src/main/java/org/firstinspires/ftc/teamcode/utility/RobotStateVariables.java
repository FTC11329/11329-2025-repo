package org.firstinspires.ftc.teamcode.utility;

public class RobotStateVariables {
    public PlacePosEnum whereAmI;
    public boolean hasInIntake;
    public boolean hasInOutake;

    public boolean clawToggle = true;
    public RobotStateVariables(PlacePosEnum whereAmI) {
        this(whereAmI, false, false);
    }
    public RobotStateVariables(PlacePosEnum whereAmI, boolean hasInIntake, boolean hasInOutake) {
        this.whereAmI = whereAmI;
        this.hasInIntake = hasInIntake;
        this.hasInOutake = hasInOutake;
    }
}
