package org.firstinspires.ftc.teamcode.utility;

public class RobotStateVariables {
    public PlacePosEnum whereAmI;
    public boolean atStorePos;
    public boolean hasInIntake;
    public boolean hasInOutake;

    public boolean clawToggle = true;
    public RobotStateVariables(PlacePosEnum whereAmI) {
        this(whereAmI, false, false, false);
    }
    public RobotStateVariables(PlacePosEnum whereAmI, boolean atStorePos, boolean hasInIntake, boolean hasInOutake) {
        this.whereAmI = whereAmI;
        this.atStorePos = atStorePos;
        this.hasInIntake = hasInIntake;
        this.hasInOutake = hasInOutake;
    }
}
