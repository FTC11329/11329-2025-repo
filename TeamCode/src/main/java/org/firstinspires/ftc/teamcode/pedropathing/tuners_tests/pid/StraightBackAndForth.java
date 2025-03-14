package org.firstinspires.ftc.teamcode.pedropathing.tuners_tests.pid;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import org.firstinspires.ftc.teamcode.pedropathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedropathing.pathgen.PathChain;
import org.firstinspires.ftc.teamcode.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedropathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedropathing.pathgen.BezierLine;
import org.firstinspires.ftc.teamcode.pedropathing.pathgen.Path;
import org.firstinspires.ftc.teamcode.pedropathing.pathgen.Point;
import org.firstinspires.ftc.teamcode.pedropathing.util.Drawing;

/**
 * This is the StraightBackAndForth autonomous OpMode. It runs the robot in a specified distance
 * straight forward. On reaching the end of the forward Path, the robot runs the backward Path the
 * same distance back to the start. Rinse and repeat! This is good for testing a variety of Vectors,
 * like the drive Vector, the translational Vector, and the heading Vector. Remember to test your
 * tunings on CurvedBackAndForth as well, since tunings that work well for straight lines might
 * have issues going in curves.
 *
 * @author Anyi Lin - 10158 Scott's Bots
 * @author Aaron Yang - 10158 Scott's Bots
 * @author Harrison Womack - 10158 Scott's Bots
 * @version 1.0, 3/12/2024
 */
@Config
@Autonomous (name = "Straight Back And Forth", group = "PIDF Tuning")
public class StraightBackAndForth extends OpMode {
    private Telemetry telemetryA;

    public static double DISTANCE = 40;

    private boolean forward = true;

    private Follower follower;

    FtcDashboard dashboard;
    private Path forwards;
    private Path backwards;
    ElapsedTime time = new ElapsedTime();

    double loopTime = time.milliseconds();
    private final Pose startPose = new Pose(9, -65.3, Math.toRadians(0));
    private final Pose placeSub1 = new Pose(15, -32.5, Math.toRadians(90));
    private final Pose frontWall  = new Pose(40, -57, Math.toRadians(90));


    /**
     * This initializes the Follower and creates the forward and backward Paths. Additionally, this
     * initializes the FTC Dashboard telemetry.
     */
    @Override
    public void init() {
        dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();


        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);

        forwards = follower.linearPathBuilder(placeSub1.addReturn(new Pose(0,-3,0)), frontWall);
        forwards.setConstantHeadingInterpolation(placeSub1.getHeading());
        forwards.setZeroPowerAccelerationMultiplier(4);

        backwards = follower.linearPathBuilder(frontWall, placeSub1.addReturn(new Pose(0,-3,0)));
        backwards.setConstantHeadingInterpolation(placeSub1.getHeading());
        backwards.setZeroPowerAccelerationMultiplier(4);

        follower.followPath(forwards);

        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetryA.addLine("This will run the robot in a straight line going " + DISTANCE
                            + " inches forward. The robot will go forward and backward continuously"
                            + " along the path. Make sure you have enough room.");
        telemetryA.update();
    }

    /**
     * This runs the OpMode, updating the Follower as well as printing out the debug statements to
     * the Telemetry, as well as the FTC Dashboard.
     */
    @Override
    public void loop() {
        follower.update();
        if (!follower.isBusy()) {
            if (forward) {
                forward = false;
                follower.followPath(backwards);
            } else {
                forward = true;
                follower.followPath(forwards);
            }
        }

        Drawing.drawDebug(follower);

        telemetry.addData("loopTime", loopTime - time.milliseconds());
        loopTime = time.milliseconds();
        telemetry.update();
    }
}
