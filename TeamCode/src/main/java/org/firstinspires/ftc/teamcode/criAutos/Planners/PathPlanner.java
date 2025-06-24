package org.firstinspires.ftc.teamcode.criAutos.Planners;

import org.firstinspires.ftc.teamcode.utility.Robot;
import org.firstinspires.ftc.teamcode.pedropathing.localization.Pose;

public interface PathPlanner {
    // Run the step, return true when it's done
    boolean run();

    // Builds all the paths with the previous offset
    void buildPaths(Pose offset);

    // Used to connect paths
    Pose getEndPose();

    // Gets the offset of the previous paths plus this offset
    Pose getOffset();

}
