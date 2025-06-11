package org.firstinspires.ftc.teamcode.criAutos.Planners;

import org.firstinspires.ftc.teamcode.utility.Robot;
import org.firstinspires.ftc.teamcode.pedropathing.localization.Pose;

public interface PathPlanner {
    // Run the step, return true when it's done
    boolean run();

    // Used to connect paths
    Pose getEndPose();
}
