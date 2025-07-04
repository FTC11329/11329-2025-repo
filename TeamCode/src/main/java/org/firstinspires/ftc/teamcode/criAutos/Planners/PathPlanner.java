package org.firstinspires.ftc.teamcode.criAutos.Planners;

import org.firstinspires.ftc.teamcode.pedropathing.localization.Pose;

public interface PathPlanner {
    // Run the step, return true when it's done
    boolean run();

    // Builds all the paths with the previous offset
    void buildPaths(Pose offset);

    // Used to connect paths
    Pose getEndPoseEst();

    // Gets the offset of the previous paths plus this offset
    Pose getOffset();

    // Set during the building of the paths to allow for easy tuning
    void addToOffset(Pose offset);

    // Gets the name of the module
    String getName();

}
