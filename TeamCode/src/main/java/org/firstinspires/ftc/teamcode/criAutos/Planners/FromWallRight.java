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
        /// Places on bar right outer with option for low or high
        /// Ends facing left sub intake at place preset
        /// Expects arm to start under the bar if high bar = false
        // Variables
        boolean highBar;
        Pose offset = new Pose();

        private Timer pathTimer;
        private int state = 0;
        private boolean isFinished = false;

        // Pass-through Variables
        private volatile Robot robot;
        private Pose startPose;
        public ToRightOuterBar(Robot robot, Pose startPose, boolean highBar, Pose offset) {
            addToOffset(offset);
            pathTimer = new Timer();
            this.robot = robot;
            this.startPose = startPose;
            this.highBar = highBar;
        }
        public ToRightOuterBar(Robot robot, Pose startPose, boolean highBar) {
            pathTimer = new Timer();
            this.robot = robot;
            this.startPose = startPose;
            this.highBar = highBar;
        }
        //Poses
        Pose controlPoint = new Pose(-39, -114);

        Pose startPoseAdded;
        Pose controlPointAdded;
        Pose barRightOuterMidAdded;
        Pose barRightOuterTopAdded;

        //Paths
        PathChain toBar;

        @Override
        public void buildPaths(Pose offset) {
            this.offset.add(offset);
//            nextOffset = this.offset + nextOffset

            startPoseAdded = startPose.addReturn(offset);
            controlPointAdded = controlPoint.addReturn(offset);
            barRightOuterMidAdded = barRightOuterMid.addReturn(offset).addReturn(thisOffset);
            barRightOuterTopAdded = barRightOuterTop.addReturn(offset).addReturn(thisOffset);

            toBar = robot.follower.pathBuilder()
                    .addPath(new Path(new BezierCurve(new Point(startPoseAdded), new Point(controlPointAdded), new Point(barRightOuterMidAdded))))
                    .setLinearHeadingInterpolation(startPoseAdded.getHeading(), barRightOuterMidAdded.getHeading(), 0.3)
                    .addPath(robot.follower.linearPathBuilder(barRightOuterMidAdded, barRightOuterTopAdded))
                    .setConstantHeadingInterpolation(barRightOuterTopAdded.getHeading())
                    .build();
        }


        @Override
        public Pose getEndPoseEst() {
            return barRightOuterMid;
        }

        @Override
        public boolean run() {
            switch (state) {
                case 0:
                    if (highBar) {
                        robot.stateMachine.goHighSpecimen(false, false);
                    } else {
                        robot.stateMachine.goLowSpecimen(false);
                    }
                    robot.outtakeSystem.setFlapsUp();
                    robot.follower.followPath(toBar);
                    setPathState(1);
                    break;
                case 1:
                    if (robot.follower.getErrorDistance(barRightOuterMidAdded) < 2) {
                        if (robot.robotState.whereAmI == PlacePosEnum.highSpecimen) {
                            robot.outtakeSystem.setWristPos(Constants.Outtake.postClipSpecimenWristHigh);
                        } else {
                            robot.outtakeSystem.setWristPos(Constants.Outtake.postClipSpecimenWristLow);
                        }
                        setPathState(2);
                    }
                    break;
                case 2:
                    if (robot.follower.getError(barRightOuterTopAdded).getX() < 1.5) {
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

        public void addToOffset(Pose offset) {
            this.offset = offset;
        }
        Pose thisOffset = new Pose(-2.34, -0.38);
        public Pose getOffset() {
            return offset.addReturn(new Pose(0, -0.59));
        }

        public String getName() {
            return "From Wall Right To Right Outer Bar, " + state;
        }
    }

    public static class ToRightInnerBar implements PathPlanner {
        /// Places on bar right inner with option for low or high
        /// Ends facing right at store preset
        /// Expects arm to start under the bar if high bar = false
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
                        //todo fix to be a safe position when the new claw is on
                        robot.stateMachine.goHighSpecimen(false, false);
                    } else {
                        robot.stateMachine.goLowSpecimen(false);
                    }
                    robot.follower.followPath(toOpen);
                    setPathState(1);
                    break;
                case 1:
                    if (robot.follower.getErrorDistance(barRightInnerBelowAdded) < 0.75) {
                        setPathState(2);
                    }
                    break;
                case 2:
                    if (pathTimer.getElapsedTimeSeconds() > 0) {
                        robot.follower.followPath(toNearBar);
                        setPathState(3);
                    }
                    break;
                case 3:
                    if (robot.follower.getError(barRightInnerLeftAdded).getX() < 5) {
                        if (highBar) {
                            robot.outtakeSystem.placePos(PlacePosEnum.highSpecimen);
                        } else {
                            robot.outtakeSystem.placePos(PlacePosEnum.lowSpecimen);
                        }
                        robot.follower.followPath(sweepBar);
                        setPathState(4);
                    }
                    break;
                case 4:
                    if (robot.follower.getError(barRightInnerMidAdded).getY() < 1.5) {
                        if (robot.robotState.whereAmI == PlacePosEnum.highSpecimen) {
                            robot.outtakeSystem.setWristPos(Constants.Outtake.postClipSpecimenWristHigh);
                        } else {
                            robot.outtakeSystem.setWristPos(Constants.Outtake.postClipSpecimenWristLow);
                        }
                        robot.outtakeSystem.setFlapsDown();
                        setPathState(5);
                    }
                    break;
                case 5:
                    if (robot.follower.getError(barRightInnerTopAdded).getX() < 1.5) {
                        robot.outtakeSystem.setClawPos(Constants.Outtake.dropClaw);
                        setPathState(6);
                        isFinished = true;
                    }
                    break;
            }

            return isFinished;
        }

        public void addToOffset(Pose offset) {
            this.offset = offset;
        }

        public void setPathState(int state) {
            this.state = state;
            pathTimer.resetTimer();
        }

        public Pose getOffset() {
            return offset.addReturn(new Pose(-1, 2.2));
        }

        public String getName() {
            return "From Wall Right To Right Inner Bar, " + state;
        }
    }
}
