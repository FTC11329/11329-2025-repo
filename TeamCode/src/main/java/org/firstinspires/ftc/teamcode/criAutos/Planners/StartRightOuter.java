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

public class StartRightOuter {
    /// NEAR BLUE BASKET
    public static class ToPlaceBasket implements PathPlanner {
        /// Ends at basket place pos with outake
        //Variables
        Pose offset;
        private Timer pathTimer;
        private int state = 0;
        private boolean isFinished = false;

        // Pass-through Variables
        private volatile Robot robot;
        private Pose startPose;
        private boolean preExtend;
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
            this.offset = offset;
            toBasket = robot.follower.linearPathBuilder(startLeftOuter, blueBasket);
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

        //todo
        public Pose getOffset() {
            return offset.addReturn(new Pose());
        }
    }

    public static class ToPlaceBar implements PathPlanner {
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
        public ToPlaceBar(Robot robot, Pose startPose, boolean highBar) {
            pathTimer = new Timer();
            this.robot = robot;
            this.startPose = startPose;
            this.highBar = highBar;
        }
        //Poses
        Pose controlPoint = new Pose(0, -96);

        //Paths
        PathChain toBar;

        @Override
        public void buildPaths(Pose offset) {
            this.offset = offset;
            toBar = robot.follower.pathBuilder()
                    .addPath(new Path(new BezierCurve(new Point(startPose), new Point(controlPoint), new Point(barRightOuterBot))))
                    .setLinearHeadingInterpolation(0, barRightOuterBot.getHeading(), 0.7)
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
                        robot.outtakeSystem.placePos(PlacePosEnum.highSpecimen);
                        robot.robotState.whereAmI = PlacePosEnum.highSpecimen;
                    } else {
                        robot.outtakeSystem.placePos(PlacePosEnum.lowSpecimen);
                        robot.robotState.whereAmI = PlacePosEnum.lowSpecimen;
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
                    if (robot.follower.getErrorDistance(barRightOuterTop) < 1.5) {
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
