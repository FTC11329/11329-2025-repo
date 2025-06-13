package org.firstinspires.ftc.teamcode.pedropathing.tuners_tests.localization;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import org.firstinspires.ftc.teamcode.pedropathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedropathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedropathing.pathgen.Path;
import org.firstinspires.ftc.teamcode.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedropathing.localization.PoseUpdater;
import org.firstinspires.ftc.teamcode.pedropathing.util.DashboardPoseTracker;
import org.firstinspires.ftc.teamcode.pedropathing.util.Drawing;

/**
 * This is the ForwardTuner OpMode. This tracks the forward movement of the robot and displays the
 * necessary ticks to inches multiplier. This displayed multiplier is what's necessary to scale the
 * robot's current distance in ticks to the specified distance in inches. So, to use this, run the
 * tuner, then pull/push the robot to the specified distance using a ruler on the ground. When you're
 * at the end of the distance, record the ticks to inches multiplier. Feel free to run multiple trials
 * and average the results. Then, input the multiplier into the forward ticks to inches in your
 * localizer of choice.
 * You can adjust the target distance on FTC Dashboard: 192/168/43/1:8080/dash
 *
 * @author Anyi Lin - 10158 Scott's Bots
 * @version 1.0, 5/6/2024
 */
@Config
@Autonomous(name = "Forward Localizer Tuner", group = ".Localization")
public class ForwardTuner extends OpMode {
    private Follower follower;
    private PoseUpdater poseUpdater;
    private DashboardPoseTracker dashboardPoseTracker;

    private Telemetry telemetryA;

    public static double DISTANCE = 120;
    private boolean debounce = false;
    Path forward;

    /**
     * This initializes the PoseUpdater as well as the FTC Dashboard telemetry.
     */
    @Override
    public void init() {
        follower = new Follower(hardwareMap);
        poseUpdater = follower.poseUpdater;

        dashboardPoseTracker = new DashboardPoseTracker(poseUpdater);

        forward = follower.linearPathBuilder(new Pose(0,0,0), new Pose(DISTANCE,0,0));

        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetryA.addLine("Pull your robot forward " + DISTANCE + " inches. Your forward ticks to inches will be shown on the telemetry.");
        telemetryA.update();

        Drawing.drawRobot(poseUpdater.getPose(), "#4CAF50");
        Drawing.sendPacket();
    }

    /**
     * This updates the robot's pose estimate, and updates the FTC Dashboard telemetry with the
     * calculated multiplier and draws the robot.
     */
    @Override
    public void loop() {
        follower.update();
        if (gamepad1.right_bumper && !debounce) {
            follower.setPose(new Pose(0,0,0));
            follower.followPath(forward);
            debounce = true;
        }
        if (!gamepad1.right_bumper) {
            debounce = false;
        }

        telemetryA.addData("distance moved", poseUpdater.getPose().getX());
        telemetryA.addLine("The multiplier will display what your forward ticks to inches should be to scale your current distance to " + DISTANCE + " inches.");
        telemetryA.addData("multiplier", DISTANCE / (poseUpdater.getPose().getX() / poseUpdater.getLocalizer().getForwardMultiplier()));
        telemetryA.update();

        Drawing.drawPoseHistory(dashboardPoseTracker, "#4CAF50");
        Drawing.drawRobot(poseUpdater.getPose(), "#4CAF50");
        Drawing.sendPacket();
    }
}
