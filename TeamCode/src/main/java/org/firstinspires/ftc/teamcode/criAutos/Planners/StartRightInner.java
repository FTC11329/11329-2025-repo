package org.firstinspires.ftc.teamcode.criAutos.Planners;

import static org.firstinspires.ftc.teamcode.criAutos.CommonPoses.*;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.pedropathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedropathing.pathgen.Path;
import org.firstinspires.ftc.teamcode.pedropathing.pathgen.PathChain;
import org.firstinspires.ftc.teamcode.pedropathing.util.Timer;
import org.firstinspires.ftc.teamcode.utility.PlacePosEnum;
import org.firstinspires.ftc.teamcode.utility.Robot;

public class StartRightInner {
    public static class ToRightInnerBar implements PathPlanner {
        /// Places on bar right inner with option for low or high
        /// Ends facing right at store preset
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
        public ToRightInnerBar(Robot robot, Pose startPose, boolean highBar) {
            pathTimer = new Timer();
            this.robot = robot;
            this.startPose = startPose;
            this.highBar = highBar;
        }
        //Poses
        //todo
        Pose startPoseAdded;
        Pose startLeftOuterAdded;
        Pose barRightInnerMidBelowAdded;
        Pose barRightInnerMidLeftAdded;
        Pose barRightInnerMidAdded;
        Pose barRightInnerTopAdded;

        //Paths
        PathChain toOpen;
        PathChain toBar;
        Path sweepBar;

        @Override
        public void buildPaths(Pose offset) {
            this.offset = offset;

            startPoseAdded = startPose.addReturn(offset);
            barRightInnerMidBelowAdded = new Pose(-32, innerSpikeRightMid.getY(), Math.toRadians(-90)).addReturn(offset);
            barRightInnerMidLeftAdded = new Pose(barRightInnerMid.getX(), innerSpikeRightMid.getY(), Math.toRadians(-90)).addReturn(offset);
            barRightInnerMidAdded = barRightInnerMid.addReturn(offset);
            barRightInnerTopAdded = barRightInnerTop.addReturn(offset);

            toOpen = robot.follower.linearPathChainBuilder(startPoseAdded, barRightInnerMidBelowAdded);

            toBar = robot.follower.pathBuilder()
                    .addPath(robot.follower.linearPathBuilder(barRightInnerMidBelowAdded, barRightInnerMidLeftAdded))
                    .addPath(robot.follower.linearPathBuilder(barRightInnerMidLeftAdded, barRightInnerMidAdded))
                    .build();

            sweepBar = robot.follower.linearPathBuilder(barRightInnerMidAdded, barRightInnerTopAdded);
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
                        //todo fix to be a safe position when the new claw is on
                        robot.stateMachine.goHighSpecimen(false, false);
                    } else {
                        robot.stateMachine.goLowSpecimen(false);
                    }
                    robot.follower.followPath(toOpen);
                    setPathState(1);
                    break;
                case 1:
                    if (robot.follower.getErrorDistance(barRightInnerMidBelowAdded) < 0.75) {
                        setPathState(2);
                    }
                    break;
                case 2:
                    if (pathTimer.getElapsedTimeSeconds() > 0.25) {
                        robot.follower.followPath(toBar);
                        setPathState(3);
                    }
                    break;
                case 3:
                    if (robot.follower.getError(barRightInnerMidAdded).getX() < 3) {
                        if (highBar) {
                            robot.outtakeSystem.placePos(PlacePosEnum.highSpecimen);
                        } else {
                            robot.outtakeSystem.placePos(PlacePosEnum.lowSpecimen);
                        }
                        setPathState(4);
                    }
                    break;
                case 4:
                    if (robot.follower.getError(barRightInnerMidAdded).getY() < 1.5) {
                        robot.outtakeSystem.setWristPos(Constants.Outtake.postClipSpecimenWrist);
                        robot.follower.followPath(sweepBar);
                        setPathState(5);
                    }
                    break;
                case 5:
                    if (robot.follower.getError(barRightInnerTopAdded).getX() < 1.5) {
                        robot.outtakeSystem.setClawPos(Constants.Outtake.grabClaw);
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

        //todo
        public Pose getOffset() {
            return offset.addReturn(new Pose(-1.1, 0.7));
        }

        public String getName() {
            return "Start right inner To place Bar" + state;
        }
    }
}
