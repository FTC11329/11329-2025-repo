package org.firstinspires.ftc.teamcode.criAutos.Planners;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.Constants;
import static org.firstinspires.ftc.teamcode.criAutos.CommonPoses.*;
import org.firstinspires.ftc.teamcode.pedropathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedropathing.pathgen.BezierCurve;
import org.firstinspires.ftc.teamcode.pedropathing.pathgen.Path;
import org.firstinspires.ftc.teamcode.pedropathing.pathgen.PathChain;
import org.firstinspires.ftc.teamcode.pedropathing.pathgen.Point;
import org.firstinspires.ftc.teamcode.pedropathing.util.Timer;
import org.firstinspires.ftc.teamcode.utility.PlacePosEnum;
import org.firstinspires.ftc.teamcode.utility.Robot;

public class StartLeftOuter {
    /// NEAR RED BASKET
    public static class ToPlaceBasket implements PathPlanner {
        /// Ends at basket place pos with outake
        //Variables
        Pose offset = new Pose();
        private Timer pathTimer;
        private int state = 0;
        private boolean isFinished = false;

        // Pass-through Variables
        private volatile Robot robot;
        private Pose startPose;
        private boolean preExtend;

        public ToPlaceBasket(Robot robot, Pose startPose, boolean preExtend, Pose offset) {
            addToOffset(offset);
            pathTimer = new Timer();
            this.robot = robot;
            this.startPose = startPose;
            this.preExtend = preExtend;
        }
        public ToPlaceBasket(Robot robot, Pose startPose, boolean preExtend) {
            pathTimer = new Timer();
            this.robot = robot;
            this.startPose = startPose;
            this.preExtend = preExtend;
        }

        //Paths
        Path toBasket;

        @Override
        public void buildPaths(Pose offset) {
            this.offset.add(offset);
            toBasket = robot.follower.linearPathBuilder(startLeftOuter, redBasket);
        }

        @Override
        public Pose getEndPoseEst() {
            return new Pose();
        }

        @Override
        public boolean run() {
            switch (state) {
                case 0:
                    robot.outtakeSystem.setArmPos(Constants.Outtake.upArm);
                    robot.outtakeSystem.setVSlidePos(Constants.Outtake.highBasketSlides);
                    robot.follower.followPath(toBasket);
                    setPathState(1);
                    break;
                case 1:
                    if (Math.abs(robot.outtakeSystem.getVSlidePos() - Constants.Outtake.highBasketSlides) < 50) {
                        robot.outtakeSystem.setArmPos(Constants.Outtake.basketArm);
                        if (preExtend) {
                            robot.intakeSystem.setHSlidePos(Constants.Intake.autoPreExtendSlides - 225);
                        }
                        setPathState(2);
                    }
                    break;
                case 2:
                    if (robot.follower.getErrorDistance(blueBasket) < 1) {
                        setPathState(3);
                    }
                    break;
                case 3:
                    if (pathTimer.getElapsedTimeSeconds() > 0.4) {
                        robot.outtakeSystem.setClawPos(Constants.Outtake.dropClaw);
                        setPathState(4);
                    }
                    break;
                case 4:
                    if (pathTimer.getElapsedTimeSeconds() > 0.25) {
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
        //todo low priority
        public Pose getOffset() {
            return offset.addReturn(new Pose());
        }

        public String getName() {
            return "Start Left Outer To place Basket" + state;
        }
    }

    public static class ToPlaceLeftOuterBar implements PathPlanner {
        /// Places on bar left outer with option for low or high
        /// Ends at bar at post clip position
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
        public ToPlaceLeftOuterBar(Robot robot, Pose startPose, boolean highBar) {
            pathTimer = new Timer();
            this.robot = robot;
            this.startPose = startPose;
            this.highBar = highBar;
        }
        //Poses
        Pose controlPoint = new Pose(0, 96);

        Pose startPoseAdded;
        Pose controlPointAdded;
        Pose barLeftOuterMidAdded;
        Pose barLeftOuterTopAdded;

        //Paths
        PathChain toBar;

        @Override
        public void buildPaths(Pose offset) {
            this.offset.add(offset);

            startPoseAdded = startPose.addReturn(offset);
            controlPointAdded = controlPoint.addReturn(offset);
            barLeftOuterMidAdded = barLeftOuterMid.addReturn(offset);
            barLeftOuterTopAdded = barLeftOuterTop.addReturn(offset);

            toBar = robot.follower.pathBuilder()
                    .addPath(new Path(new BezierCurve(new Point(startPoseAdded), new Point(controlPointAdded), new Point(barLeftOuterMidAdded))))
                    .setConstantHeadingInterpolation(Math.toRadians(-90))
                    .addPath(robot.follower.linearPathBuilder(barLeftOuterMidAdded, barLeftOuterTopAdded))
                    .setConstantHeadingInterpolation(barLeftOuterMidAdded.getHeading())
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
                    robot.follower.followPath(toBar);
                    if (highBar) {
                        robot.outtakeSystem.placePos(PlacePosEnum.highSpecimen);
                        robot.robotState.whereAmI = PlacePosEnum.highSpecimen;
                    } else {
                        robot.outtakeSystem.placePos(PlacePosEnum.lowSpecimen);
                        robot.robotState.whereAmI = PlacePosEnum.lowSpecimen;
                    }
                    setPathState(1);
                    break;
                case 1:
                    if (robot.follower.getError(barLeftOuterMidAdded).getX() < 3) {
                        if (robot.robotState.whereAmI == PlacePosEnum.highSpecimen) {
                            robot.outtakeSystem.placePos(PlacePosEnum.postClipHighSpecimen);
                        } else {
                            robot.outtakeSystem.placePos(PlacePosEnum.postClipLowSpecimen);
                        }
                        setPathState(2);
                    }
                    break;
                case 2:
                    if (robot.follower.getErrorDistance(barLeftOuterTopAdded) < 1.5) {
                        robot.outtakeSystem.setClawPos(Constants.Outtake.dropClaw);
                        setPathState(3);
                    }
                    break;
                case 3:
                    if (pathTimer.getElapsedTimeSeconds() > 0.3) {
                        setPathState(4);
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

        // todo
        public Pose getOffset() {
            return offset.addReturn(new Pose(1.7, -2.15));
        }

        public String getName() {
            return "Start left outer To place Bar" + state;
        }
    }
}
