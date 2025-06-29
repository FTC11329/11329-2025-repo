package org.firstinspires.ftc.teamcode.criAutos.Planners;

import static org.firstinspires.ftc.teamcode.criAutos.CommonPoses.*;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.pedropathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedropathing.pathgen.BezierCurve;
import org.firstinspires.ftc.teamcode.pedropathing.pathgen.Path;
import org.firstinspires.ftc.teamcode.pedropathing.pathgen.PathBuilder;
import org.firstinspires.ftc.teamcode.pedropathing.pathgen.PathChain;
import org.firstinspires.ftc.teamcode.pedropathing.pathgen.Point;
import org.firstinspires.ftc.teamcode.pedropathing.util.Timer;
import org.firstinspires.ftc.teamcode.utility.PlacePosEnum;
import org.firstinspires.ftc.teamcode.utility.Robot;
import org.firstinspires.ftc.teamcode.utility.autoEnums.Specimen6AutoEnum;

public class FromBarRightOuter {
    public static class ToWallSuper implements PathPlanner {
        /// Option To go left or right wall
        /// expects robot.state.whereami to be correct
        /// picks one up and transfers it and puts it in the o-zone
        // Variables
        boolean rightWall;
        Pose offset;

        Pose2D visionResult;
        private Timer pathTimer;
        private int state = 0;
        private boolean isFinished = false;

        // Pass-through Variables
        private volatile Robot robot;
        private Pose startPose;
        public ToWallSuper(Robot robot, Pose startPose, boolean rightWall) {
            pathTimer = new Timer();
            this.robot = robot;
            this.startPose = startPose;
            this.rightWall = rightWall;
        }
        //Poses
        //todo
        Pose controlPoint = new Pose(-24, -110);
        Pose midPoint = new Pose(-48, -48, Math.toRadians(-75));

        //Paths
        Path toBasket;
        PathChain toWall;
        PathBuilder toWallBuilder;

        @Override
        public void buildPaths(Pose offset) {
            this.offset = offset;
            toBasket = robot.follower.linearPathBuilder(startLeftOuter, redBasket);
            toWallBuilder = robot.follower.pathBuilder()
                    .addPath(new Path(new BezierCurve(new Point(barRightOuterTop), new Point(controlPoint), new Point(midPoint))))
                    .setLinearHeadingInterpolation(barRightOuterTop.getHeading(), midPoint.getHeading(), 0.7);

            if (rightWall) {
                toWallBuilder.addPath(robot.follower.linearPathBuilder(midPoint, pickupWallRight))
                        .setLinearHeadingInterpolation(midPoint.getHeading(), pickupWallRight.getHeading(), 0.9);
            } else {
                toWallBuilder.addPath(robot.follower.linearPathBuilder(midPoint, pickupWallLeft))
                        .setLinearHeadingInterpolation(midPoint.getHeading(), pickupWallLeft.getHeading(), 0.9);
            }
            toWall = toWallBuilder.build();
        }

        @Override
        public Pose getEndPoseEst() {
            if (rightWall) {
                return pickupWallRight;
            } else {
                return pickupWallLeft;
            }
        }

        @Override
        public boolean run() {
            switch (state) {
                case 0:
                    if (robot.robotState.whereAmI == PlacePosEnum.highSpecimen) {
                        robot.outtakeSystem.setVSlidePos(Constants.Outtake.safeFromSpecBar);
                    }
                    visionResult = robot.blockVision.getBlockPosition(true);
                    if (visionResult.getHeading(AngleUnit.DEGREES) != -1) {
                        robot.outtakeSystem.setClawPos(Constants.Outtake.dropClaw);
                        robot.intakeSystem.setHSlidesInches(robot.follower.followYourHead(visionResult));
                        setPathState(1);

                    } else if (pathTimer.getElapsedTimeSeconds() > 1) {
                        robot.outtakeSystem.placePos(PlacePosEnum.wall);
                        robot.outtakeSystem.setClawPos(Constants.Outtake.dropClaw);
                        robot.intakeSystem.storePos();
                        robot.follower.followPath(toWall);
                        setPathState(6);
                    }
                    break;
                case 1:
                    if ((pathTimer.getElapsedTimeSeconds() > 0.2 && Math.abs(robot.intakeSystem.getHSlideTargetPos() - robot.intakeSystem.getHSlidePos()) < 250 && robot.follower.getHeadingError() < Math.toRadians(2)) || pathTimer.getElapsedTimeSeconds() > 0.5) {
                        robot.intakeSystem.setIntakeServoPos(Constants.Intake.wristDown);
                        if (robot.robotState.whereAmI == PlacePosEnum.highSpecimen) {
                            robot.outtakeSystem.setArmPos(Constants.Outtake.intakeArm);
                        }
                        setPathState(2);
                    }
                    break;
                case 2:
                    if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                        robot.stateMachine.goStore();
                        robot.doDriveShake = true;
                        robot.follower.startTeleopDrive();
                        robot.intakeSystem.intakeUntilColor();
                        setPathState(3);
                    }
                    break;
                case 3:
                    robot.intakeSystem.update();
                    if (robot.intakeSystem.intakeUntilColor()) {

                        robot.intakeSystem.storeOutPos();
                        robot.intakeSystem.setIntakePower(Constants.Intake.unjamSpeed);
                        robot.doDriveShake = false;
                        robot.follower.breakFollowing();
                        robot.robotState.hasInIntake = true;

                        robot.follower.followPath(toWall);
                        setPathState(4);

                    } else if (pathTimer.getElapsedTimeSeconds() > 2.5) {
                        robot.intakeSystem.storeOutPos();
                        robot.intakeSystem.setIntakePower(Constants.Intake.spitSpeed);
                        robot.follower.breakFollowing();
                        robot.doDriveShake = false;

                        setPathState(6);
                    }
                    break;
                case 4:
                    if (pathTimer.getElapsedTimeSeconds() > Constants.Intake.unjamTimeMillisAuto / 1000) {
                        robot.intakeSystem.setIntakePower(Constants.Intake.intakeSpeed);
                        setPathState(5);
                    }
                    break;
                case 5:
                    if (robot.intakeSystem.intakeUntil() || pathTimer.getElapsedTimeSeconds() > Constants.Intake.unjamTimeMillisAuto / 1000){
                        robot.intakeSystem.setIntakePower(0);
                        robot.telemetry.addData("wall", true);
                        robot.stateMachine.goWall(true, robot.robotState.whereAmI == PlacePosEnum.lowSpecimen, false);
                        setPathState(7);
                    }
                    break;
                    //Failed pickup
                case 6:
                    robot.follower.followPath(toWall);
                    robot.stateMachine.goWall(false, robot.robotState.whereAmI == PlacePosEnum.lowSpecimen, robot.robotState.atStorePos);
                    setPathState(7);
                    break;
                case 7:
                    robot.telemetry.addData("wall", true);
                    if (robot.follower.getError(pickupWallRight).getY() < 2) {
                        robot.outtakeSystem.setClawPos(Constants.Outtake.dropClaw);
                        setPathState(8);
                    }
                    break;
                case 8:
                    if (robot.outtakeSystem.seesWall() || pathTimer.getElapsedTimeSeconds() > 5) {
                        setPathState(9);
                    }
                    break;
                case 9:
                    if (pathTimer.getElapsedTimeSeconds() > 0.05) {
                        robot.outtakeSystem.setClawPos(Constants.Outtake.grabClaw);
                        setPathState(10);
                    }
                    break;
                case 10:
                    if (pathTimer.getElapsedTimeSeconds() > 0.3) {
                        isFinished = true;
                        setPathState(11);
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
