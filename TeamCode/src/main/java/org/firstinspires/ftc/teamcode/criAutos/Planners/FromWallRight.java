package org.firstinspires.ftc.teamcode.criAutos.Planners;

import static org.firstinspires.ftc.teamcode.criAutos.CommonPoses.*;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.pedropathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedropathing.pathgen.BezierCurve;
import org.firstinspires.ftc.teamcode.pedropathing.pathgen.Path;
import org.firstinspires.ftc.teamcode.pedropathing.pathgen.PathChain;
import org.firstinspires.ftc.teamcode.pedropathing.pathgen.Point;
import org.firstinspires.ftc.teamcode.pedropathing.util.Timer;
import org.firstinspires.ftc.teamcode.utility.PlacePosEnum;
import org.firstinspires.ftc.teamcode.utility.Robot;

public class FromWallRight {
    public static class ToRightOuterBar implements PathPlanner {
        /// Places on bar left inner with option for low or high
        /// Ends facing left sub intake at store preset
        /// Expects arm to start under the bar if high bar = false
        // Variables
        boolean highBar;
        Pose offset;
        private Timer pathTimer;
        private int state = 0;
        private boolean isFinished = false;

        // Pass-through Variables
        private volatile Robot robot;
        private Pose startPose;
        public ToRightOuterBar(Robot robot, Pose startPose, boolean highBar) {
            pathTimer = new Timer();
            this.robot = robot;
            this.startPose = startPose;
            this.highBar = highBar;
        }
        //Poses
        Pose controlPoint = new Pose(-36, -120);

        //Paths
        PathChain toBar;

        @Override
        public void buildPaths(Pose offset) {
            this.offset = offset;

            toBar = robot.follower.pathBuilder()
                    .addPath(new Path(new BezierCurve(new Point(startPose), new Point(controlPoint), new Point(barRightOuterBot))))
                    .setTangentHeadingInterpolation()
                    .addPath(robot.follower.linearPathBuilder(barRightOuterBot, barRightOuterTop))
                    .setConstantHeadingInterpolation(barRightOuterBot.getHeading())
                    .build();
        }

        @Override
        public Pose getEndPoseEst() {
            return barRightOuterTop;
        }

        @Override
        public boolean run() {
            switch (state) {
                case 0:
                    robot.follower.followPath(toBar);
                    if (highBar) {
                        robot.stateMachine.goHighSpecimen(false, false);
                    } else {
                        robot.stateMachine.goLowSpecimen(false);
                    }
                    setPathState(1);
                    break;
                case 1:
                    if (robot.follower.getError(barRightOuterMid).getX() < 3) {
                        robot.outtakeSystem.setWristPos(Constants.Outtake.postClipSpecimenWrist);
                        setPathState(2);
                    }
                    break;
                case 2:
                    if (robot.follower.getErrorDistance(barRightOuterTop) < 2) {
                        robot.outtakeSystem.setClawPos(Constants.Outtake.dropClaw);
                        setPathState(3);
                        isFinished = true;
                    }
                    break;

            }

            return isFinished;
        }

        public void setPathState(int state) {
            this.state = state;
            pathTimer.resetTimer();
        }

        //todo
        public Pose getOffset() {
            return offset.addReturn(new Pose());
        }
    }
}
