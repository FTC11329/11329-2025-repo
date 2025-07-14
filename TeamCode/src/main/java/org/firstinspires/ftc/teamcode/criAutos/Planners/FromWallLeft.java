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

public class FromWallLeft {
    public static class ToLeftOuterBar implements PathPlanner {
        /// Places on bar left outer with option for low or high
        /// Ends at bar at place preset
        // Variables
        boolean highBar;
        Pose offset = new Pose();

        private Timer pathTimer;
        private int state = 0;
        private boolean isFinished = false;

        // Pass-through Variables
        private volatile Robot robot;
        private Pose startPose;
        public ToLeftOuterBar(Robot robot, Pose startPose, boolean highBar, Pose offset) {
            addToOffset(offset);
            pathTimer = new Timer();
            this.robot = robot;
            this.startPose = startPose;
            this.highBar = highBar;
        }
        public ToLeftOuterBar(Robot robot, Pose startPose, boolean highBar) {
            pathTimer = new Timer();
            this.robot = robot;
            this.startPose = startPose;
            this.highBar = highBar;
        }
        //Poses
        Pose controlPoint = new Pose(-39, 114);

        Pose startPoseAdded;
        Pose controlPointAdded;
        Pose barLeftOuterMidAdded;
        Pose barLeftOuterTopAdded;

        //Paths
        PathChain toBar;

        @Override
        public void buildPaths(Pose offset) {
            this.offset.add(offset);
//            nextOffset = this.offset + nextOffset

            startPoseAdded = startPose.addReturn(offset);
            controlPointAdded = controlPoint.addReturn(offset);
            barLeftOuterMidAdded = barLeftOuterMid.addReturn(offset).addReturn(thisOffset);
            barLeftOuterTopAdded = barLeftOuterTop.addReturn(offset).addReturn(thisOffset);

            toBar = robot.follower.pathBuilder()
                    .addPath(new Path(new BezierCurve(new Point(startPoseAdded), new Point(controlPointAdded), new Point(barLeftOuterMidAdded))))
                    .setLinearHeadingInterpolation(startPoseAdded.getHeading(), barLeftOuterMidAdded.getHeading(), 0.3)
                    .addPath(robot.follower.linearPathBuilder(barLeftOuterMidAdded, barLeftOuterTopAdded))
                    .setConstantHeadingInterpolation(barLeftOuterTopAdded.getHeading())
                    .build();
        }


        @Override
        public Pose getEndPoseEst() {
            return barLeftOuterMid;
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
                    if (robot.follower.getErrorDistance(barLeftOuterMidAdded) < 2) {
                        if (robot.robotState.whereAmI == PlacePosEnum.highSpecimen) {
                            robot.outtakeSystem.placePos(PlacePosEnum.postClipHighSpecimen);
                        } else {
                            robot.outtakeSystem.placePos(PlacePosEnum.postClipLowSpecimen);
                        }
                        setPathState(2);
                    }
                    break;
                case 2:
                    if (robot.follower.getError(barLeftOuterTopAdded).getX() < 1.5) {
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
        //todo
        Pose thisOffset = new Pose();
        //todo
        public Pose getOffset() {
            return offset.addReturn(new Pose());
        }

        public String getName() {
            return "From Wall Left To Left Outer Bar, " + state;
        }
    }
    public static class ToLeftInnerBar implements PathPlanner {
        /// Places on bar left inner with option for low or high
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
        public ToLeftInnerBar(Robot robot, Pose startPose, boolean highBar, Pose offset) {
            addToOffset(offset);
            pathTimer = new Timer();
            this.robot = robot;
            this.startPose = startPose;
            this.highBar = highBar;
        }
        public ToLeftInnerBar(Robot robot, Pose startPose, boolean highBar) {
            pathTimer = new Timer();
            this.robot = robot;
            this.startPose = startPose;
            this.highBar = highBar;
        }
        //Poses
        Pose startPoseAdded;
        Pose barLeftInnerBelowAdded;
        Pose barLeftInnerRightAdded;
        Pose barLeftInnerMidAdded;
        Pose barLeftInnerTopAdded;

        //Paths
        PathChain toOpen;
        PathChain toNearBar;
        PathChain sweepBar;

        @Override
        public void buildPaths(Pose offset) {
            this.offset.add(offset);

            startPoseAdded = startPose.addReturn(offset);
            barLeftInnerBelowAdded = new Pose(-32, innerSpikeLeftMid.getY(), Math.toRadians(90)).addReturn(offset);
            barLeftInnerRightAdded = new Pose(barLeftInnerBot.getX(), innerSpikeLeftBot.getY(), Math.toRadians(90)).addReturn(offset);
            barLeftInnerMidAdded = barLeftInnerMid.addReturn(offset);
            barLeftInnerTopAdded = barLeftInnerTop.addReturn(offset);

            toOpen = robot.follower.linearPathChainBuilder(startPoseAdded, barLeftInnerBelowAdded);

            toNearBar = robot.follower.linearPathChainBuilder(barLeftInnerBelowAdded, barLeftInnerRightAdded);

            sweepBar = robot.follower.pathBuilder()
                    .addPath(robot.follower.linearPathBuilder(barLeftInnerRightAdded, barLeftInnerMidAdded))
                    .addPath(robot.follower.linearPathBuilder(barLeftInnerMidAdded, barLeftInnerTopAdded))
                    .build();
        }

        @Override
        public Pose getEndPoseEst() {
            return barLeftInnerMid;
        }

        @Override
        public boolean run() {
            switch (state) {
                case 0:
                    if (highBar) {
                        robot.stateMachine.goHighSpecimen(false, false);
                        robot.stateMachine.setAutoPresets(true);
                    } else {
                        robot.stateMachine.goLowSpecimen(false);
                        robot.stateMachine.setAutoPresets(true);
                    }
                    robot.follower.followPath(toOpen);
                    setPathState(1);
                    break;
                case 1:
                    if (robot.follower.getErrorDistance(barLeftInnerBelowAdded) < 0.75) {
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
                    if (robot.follower.getError(barLeftInnerRightAdded).getX() < 5) {
                        robot.follower.followPath(sweepBar);
                        setPathState(4);
                    }
                    break;
                case 4:
                    if (robot.follower.getError(barLeftInnerMidAdded).getY() < 1.5) {
                        if (robot.robotState.whereAmI == PlacePosEnum.highSpecimen) {
                            robot.outtakeSystem.placePos(PlacePosEnum.postClipHighSpecimenAuto);
                        } else {
                            robot.outtakeSystem.placePos(PlacePosEnum.postClipLowSpecimenAuto);
                        }
                        robot.outtakeSystem.setFlapsWall();
                        setPathState(5);
                    }
                    break;
                case 5:
                    if (robot.follower.getError(barLeftInnerTopAdded).getX() < 1.5) {
                        if (robot.robotState.whereAmI == PlacePosEnum.lowSpecimen) {
                            robot.outtakeSystem.setArmPos(Constants.Outtake.lowSpecimenArmAutoPost);
                        }
                        setPathState(6);
                    }
                    break;
                case 6:
                    if (pathTimer.getElapsedTimeSeconds() > 0.25) {
                        robot.outtakeSystem.setClawPos(Constants.Outtake.dropClaw);
                        setPathState(7);
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
            return offset.addReturn(new Pose(0.3, -0.95));
        }

        public String getName() {
            return "From Wall Left To Left Inner Bar, " + state;
        }
    }
}
