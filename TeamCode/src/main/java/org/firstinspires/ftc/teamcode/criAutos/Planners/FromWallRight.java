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
        Pose controlPoint = new Pose(-32, -110);

        Pose startPoseAdded;
        Pose controlPointAdded;
        Pose barRightOuterMidShifted;
        Pose barRightOuterMidAdded;
        Pose barRightOuterBotAdded;

        //Paths
        PathChain toBar;

        @Override
        public void buildPaths(Pose offset) {
            this.offset = offset;

            startPoseAdded = startPose.addReturn(offset);
            controlPointAdded = controlPoint.addReturn(offset);
            barRightOuterMidShifted = barRightOuterMid.addReturn(new Pose(0, -15)).addReturn(offset);
            barRightOuterMidAdded = barRightOuterMid.addReturn(offset);
            barRightOuterBotAdded = barRightOuterBot.addReturn(offset);

            toBar = robot.follower.pathBuilder()
                    .addPath(new Path(new BezierCurve(new Point(startPoseAdded), new Point(controlPointAdded), new Point(barRightOuterMidShifted))))
                    .setTangentHeadingInterpolation()
                    .addPath(robot.follower.linearPathBuilder(barRightOuterMidShifted, barRightOuterMidAdded))
                    .setConstantHeadingInterpolation(barRightOuterBotAdded.getHeading())
                    .addPath(robot.follower.linearPathBuilder(barRightOuterBotAdded, barRightOuterMidAdded))
                    .setConstantHeadingInterpolation(barRightOuterBotAdded.getHeading())
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
                    robot.follower.followPath(toBar);
                    setPathState(1);
                    break;
                case 1:
                    if (robot.follower.getErrorDistance(barRightOuterMidAdded) < 2) {
                        robot.outtakeSystem.setWristPos(Constants.Outtake.postClipSpecimenWrist);
                        setPathState(2);
                    }
                    break;
                case 2:
                    if (pathTimer.getElapsedTimeSeconds() > 0.5) {
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

            toBar = robot.follower.pathBuilder()
                    .addPath(robot.follower.linearPathBuilder(startPoseAdded, barRightInnerMidBelowAdded))
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
                    robot.follower.followPath(toBar);
                    setPathState(1);
                    break;
                case 1:
                    if (robot.follower.getError(barRightInnerMidAdded).getX() < 3) {
                        if (highBar) {
                            robot.outtakeSystem.placePos(PlacePosEnum.highSpecimen);
                        } else {
                            robot.outtakeSystem.placePos(PlacePosEnum.lowSpecimen);
                        }
                        setPathState(2);
                    }
                    break;
                case 2:
                    if (robot.follower.getError(barRightInnerMidAdded).getY() < 1.5) {
                        robot.outtakeSystem.setWristPos(Constants.Outtake.postClipSpecimenWrist);
                        robot.follower.followPath(sweepBar);
                        setPathState(3);
                    }
                    break;
                case 3:
                    if (robot.follower.getError(barRightInnerTopAdded).getX() < 1.5) {
                        robot.outtakeSystem.setClawPos(Constants.Outtake.grabClaw);
                        isFinished = true;
                        setPathState(4);
                    }
                    break;
            }

            return isFinished;
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
