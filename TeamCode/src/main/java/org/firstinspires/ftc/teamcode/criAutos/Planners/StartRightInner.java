package org.firstinspires.ftc.teamcode.criAutos.Planners;

import static org.firstinspires.ftc.teamcode.criAutos.CommonPoses.*;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.pedropathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedropathing.pathgen.PathChain;
import org.firstinspires.ftc.teamcode.pedropathing.util.Timer;
import org.firstinspires.ftc.teamcode.utility.PlacePosEnum;
import org.firstinspires.ftc.teamcode.utility.Robot;

public class StartRightInner {
    public static class ToRightInnerBar implements PathPlanner {
        /// Places on bar right inner with option for low or high
        /// Ends at bar at post clip preset
        // Variables
        boolean highBar;
        Pose offset = new Pose();
        private Timer pathTimer;
        private int state = 0;
        private boolean isFinished = false;

        // Pass-through Variables
        private volatile Robot robot;
        private Pose startPose;
        public ToRightInnerBar(Robot robot, Pose startPose, boolean highBar, Pose offset) {
            addToOffset(offset);
            pathTimer = new Timer();
            this.robot = robot;
            this.startPose = startPose;
            this.highBar = highBar;
        }
        public ToRightInnerBar(Robot robot, Pose startPose, boolean highBar) {
            pathTimer = new Timer();
            this.robot = robot;
            this.startPose = startPose;
            this.highBar = highBar;
        }
        //Poses
        Pose startPoseAdded;
        Pose barRightInnerBelowAdded;
        Pose barRightInnerLeftAdded;
        Pose barRightInnerMidAdded;
        Pose barRightInnerTopAdded;

        //Paths
        PathChain toOpen;
        PathChain toNearBar;
        PathChain sweepBar;

        @Override
        public void buildPaths(Pose offset) {
            this.offset.add(offset);

            startPoseAdded = startPose.addReturn(offset);
            barRightInnerBelowAdded = new Pose(-32, innerSpikeRightMid.getY(), Math.toRadians(-90)).addReturn(offset);
            barRightInnerLeftAdded = new Pose(barRightInnerBot.getX(), innerSpikeRightBot.getY(), Math.toRadians(-90)).addReturn(offset);
            barRightInnerMidAdded = barRightInnerMid.addReturn(offset);
            barRightInnerTopAdded = barRightInnerTop.addReturn(offset);

            toOpen = robot.follower.linearPathChainBuilder(startPoseAdded, barRightInnerBelowAdded);

            toNearBar = robot.follower.linearPathChainBuilder(barRightInnerBelowAdded, barRightInnerLeftAdded);

            sweepBar = robot.follower.pathBuilder()
                    .addPath(robot.follower.linearPathBuilder(barRightInnerLeftAdded, barRightInnerMidAdded))
                    .addPath(robot.follower.linearPathBuilder(barRightInnerMidAdded, barRightInnerTopAdded))
                    .build();
        }

        @Override
        public Pose getEndPoseEst() {
            return barRightInnerMid;
        }

        @Override
        public boolean run() {
            switch (state) {
                case 0:
                    if (highBar) {
                        robot.stateMachine.goHighSpecimen(false, false);
                        robot.stateMachine.setAutoPresets(true);
                    } else {
                        robot.outtakeSystem.placePos(PlacePosEnum.preClipLowSpecimenAuto);
                    }
                    robot.follower.followPath(toOpen);
                    setPathState(1);
                    break;
                case 1:
                    if (robot.follower.getErrorDistance(barRightInnerBelowAdded) < 1.5) {
                        robot.follower.followPath(toNearBar);
                        setPathState(2);
                    }
                    break;
                case 2:
                    if (robot.follower.getError(barRightInnerLeftAdded).getX() < 3) {
                        robot.follower.followPath(sweepBar);
                        setPathState(3);
                    }
                    break;
                case 3:
                    if (robot.follower.getError(barRightInnerMidAdded).getY() < 1.5) {
                        if (robot.robotState.whereAmI == PlacePosEnum.highSpecimen) {
                            robot.outtakeSystem.placePos(PlacePosEnum.postClipHighSpecimenAuto);
                        } else {
                            robot.outtakeSystem.placePos(PlacePosEnum.postClipLowSpecimenAuto);
                        }
                        robot.outtakeSystem.setFlapsWall();
                        setPathState(4);
                    }
                    break;
                case 4:
                    if (robot.follower.getError(barRightInnerTopAdded).getX() < 2) {
                        robot.outtakeSystem.setVSlidePos(Constants.Outtake.lowSpecimenSlidesAutoPost + 100);
                        setPathState(5);
                    }
                    break;
                case 5:
                    if (pathTimer.getElapsedTimeSeconds() > 0.25) {
                        robot.outtakeSystem.setClawPos(Constants.Outtake.dropClaw);
                        setPathState(6);
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
        public void addToOffset(Pose offset) {
            this.offset = offset;
        }

        public Pose getOffset() {
            return offset.addReturn(new Pose(0.25, -1.1));
        }

        public String getName() {
            return "Start right inner To place Bar" + state;
        }
    }
}
